# -*- coding: utf-8 -*-

from time import sleep
import rospy
from mavros_msgs.msg import Waypoint, WaypointList, ExtendedState, CommandCode
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPush, CommandBool, CommandTOL
from sensor_msgs.msg import NavSatFix, Imu
from de_msgs.msg import Mission
from std_msgs.msg import Empty


class FlightMission(object):
    lat = 0.
    lon = 0.
    alt = 0.

    def __init__(self):
        rospy.init_node('de_airsense_mission')
        rospy.loginfo('Waiting mavros for services...')

        self.__last_state = ExtendedState.LANDED_STATE_ON_GROUND

        self.__gpsPublisher = rospy.Publisher('de/drone/gps_position', NavSatFix, queue_size=100)
        self.__imuPublisher = rospy.Publisher('de/drone/imu', Imu, queue_size=100)
        self.__landed_pub = rospy.Publisher('de/drone/landing', Empty, queue_size=100)
        self.__takeoff_pub = rospy.Publisher('de/drone/takeoff', Empty, queue_size=100)

        def push_mission(waypoints):
            rospy.wait_for_service('mavros/mission/clear')
            try:
                service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
                service()
            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: {0}'.format(e))
            rospy.wait_for_service('mavros/mission/push')
            try:
                service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
                service(0, waypoints)
            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: {0}'.format(e))

        def set_mode(mode):
            rospy.wait_for_service('mavros/set_mode')
            try:
                service = rospy.ServiceProxy('mavros/set_mode', SetMode)
                service(0, mode)
            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: {0}'.format(e))

        def arming():
            rospy.wait_for_service('mavros/cmd/arming')
            try:
                service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                service(True)
            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: {0}'.format(e))

        def mission_create(mission_msg):
            rospy.loginfo('Waypoints from message server: ')
            rospy.loginfo(mission_msg)

            wl = WaypointList()
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL
            wp.command = CommandCode.NAV_TAKEOFF  # takeoff
            wp.is_current = True
            wp.autocontinue = True
            wp.param1 = mission_msg.waypoints[0].altitude # takeoff altitude
            wp.param2 = 0
            wp.param3 = 0
            wp.param4 = 0
            wp.x_lat =  self.lat
            wp.y_long = self.lon
            wp.z_alt = mission_msg.waypoints[0].altitude
            wl.waypoints.append(wp)

            for item in mission_msg.waypoints:
                wp = Waypoint()
                wp.frame = Waypoint.FRAME_GLOBAL
                wp.command = CommandCode.NAV_WAYPOINT  # simple point
                wp.is_current = False
                wp.autocontinue = True
                wp.param1 = 0  # takeoff altitude
                wp.param2 = 0
                wp.param3 = 0
                wp.param4 = 0

                wp.x_lat = item.latitude
                wp.y_long = item.longitude
                wp.z_alt = item.altitude
                wl.waypoints.append(wp)

            # wp = Waypoint()
            # wp.frame = 2
            # wp.command = CommandCode.NAV_RETURN_TO_LAUNCH
            # wp.is_current = False
            # wp.autocontinue = True
            # wl.waypoints.append(wp)

            push_mission(wl.waypoints)

        def mission_start ():
            # Set manual mode
            set_mode('ACRO')
            # Enable motors 
            arming()
            # Set autopilot mode
            set_mode('AUTO.TAKEOFF')
            sleep(3)
            set_mode('AUTO.MISSION')

        def gps_cb(data): 
            self.lat = data.latitude
            self.lon = data.longitude
            self.alt = data.altitude
            rospy.loginfo_throttle(1, 'position: %s, %s, %s' % (self.lat, self.lon, self.alt))
            self.__gpsPublisher.publish(data)

        rospy.Subscriber("mavros/global_position/global", NavSatFix, gps_cb)

        def imu_cb(data):
            self.__imuPublisher.publish(data)

        rospy.Subscriber("mavros/imu/data", Imu, imu_cb)

        def mission_cb(mission_msg):
            mission_create(mission_msg)
            rospy.loginfo('Mission created')
            mission_start()
            rospy.loginfo('Flight!')

        rospy.Subscriber('de/drone/mission', Mission, mission_cb)

        def state_cb(msg):
            rospy.loginfo(msg.landed_state)
            if msg.landed_state != self.__last_state:
                if msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                    self.__landed_pub.publish(Empty())
                    rospy.loginfo('landing detected')
                elif msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
                    self.__takeoff_pub.publish(Empty())
                    rospy.loginfo('takeoff detected')

                self.__last_state = msg.landed_state

        rospy.Subscriber('mavros/extended_state', ExtendedState, state_cb)

        def land_cb(msg):
            rospy.loginfo('land cmd issued')
            sleep(3)

            try:
                land_service = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
                land_service(0, 0, self.lat, self.lon, 0)
            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: {0}'.format(e))

        def takeoff_cb(msg):
            rospy.loginfo('takeoff cmd issued')
            set_mode('AUTO.TAKEOFF')
            sleep(5)
            set_mode('AUTO.MISSION')

        rospy.Subscriber('de/drone/cmd/land', Empty, land_cb)
        rospy.Subscriber('de/drone/cmd/takeoff', Empty, takeoff_cb)

    def spin(self):
        rospy.spin()
