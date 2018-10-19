# -*- coding: utf-8 -*-

from time import sleep
import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix, Imu
from de_msgs.msg import Mission

class FlightMission(object):

    MAV_GLOBAL_FRAME = 3
    MAV_CMD_WAYPOINT = 16
    MAV_CMD_RTL = 20
    MAV_CMD_LAND = 21
    MAV_CMD_TAKEOFF = 22

    lat = 0.0
    lon = 0.0

    def __init__(self):
        rospy.init_node('de_airsense_mission')
        rospy.loginfo('Waiting mavros for services...')

        self.__gpsPublisher = rospy.Publisher('de/drone/gps_position', NavSatFix, queue_size=100)
        self.__imuPublisher = rospy.Publisher('de/drone/imu', Imu, queue_size=100)

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
            wp.frame = self.MAV_GLOBAL_FRAME
            wp.command = self.MAV_CMD_TAKEOFF  # takeoff
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
                wp.frame = self.MAV_GLOBAL_FRAME
                wp.command = self.MAV_CMD_WAYPOINT  # simple point
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

            wp = Waypoint()
            wp.frame = 2 
            wp.command = self.MAV_CMD_RTL
            wp.is_current = False
            wp.autocontinue = True
            wl.waypoints.append(wp)

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
            rospy.loginfo_throttle(1, 'position: %s, %s' % (self.lat, self.lon))
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

    def spin(self):
        rospy.spin()
