# -*- coding: utf-8 -*-

import rospy
from dji_sdk.srv import SDKControlAuthority, MissionWpUpload, MissionWpAction
from dji_sdk.msg import MissionWaypointAction, MissionWaypointTask, MissionWaypoint
from de_msgs.msg import Mission
from std_msgs.msg import Empty
from sensor_msgs.msg import NavSatFix, Imu

IN_AIR_STANDBY = 3
ON_GROUND = 1


def new_mission_cb(mission_msg):
    try:
        auth = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)
        rospy.loginfo('Service. Sdk control authority:')
        resp = auth(1)
        rospy.loginfo(resp.result)
        if not resp.result:
            return
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        return

    try:
        mission = rospy.ServiceProxy('/dji_sdk/mission_waypoint_upload', MissionWpUpload)
        mission_task_msg = MissionWaypointTask()
        mission_task_msg.velocity_range     = 10;
        mission_task_msg.idle_velocity      = 10;
        mission_task_msg.action_on_finish   = MissionWaypointTask.FINISH_RETURN_TO_HOME;
        mission_task_msg.mission_exec_times = 1;
        mission_task_msg.yaw_mode           = MissionWaypointTask.YAW_MODE_AUTO;
        mission_task_msg.trace_mode         = MissionWaypointTask.TRACE_POINT;
        mission_task_msg.action_on_rc_lost  = MissionWaypointTask.ACTION_AUTO;
        mission_task_msg.gimbal_pitch_mode  = MissionWaypointTask.GIMBAL_PITCH_FREE;

        rospy.loginfo('Received mission from objective:')
        rospy.loginfo(mission_msg)
        for item in mission_msg.waypoints:
            cmd_parameter = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            mission_task_msg.mission_waypoint.append(MissionWaypoint(
                latitude = item.latitude,
                longitude = item.longitude,
                altitude = item.altitude,
                damping_distance = 2,
                target_yaw = 0,
                has_action = 1,
                target_gimbal_pitch = 0,
                turn_mode = 0,
                action_time_limit = 64000,
                waypoint_action = MissionWaypointAction(
                    action_repeat = 10,
                    command_parameter = cmd_parameter )))

        rospy.logdebug(mission_task_msg)
        rospy.loginfo('Service. Mission waypoint upload:')
        resp = mission(mission_task_msg)
        rospy.loginfo(resp.result)
        if not resp.result:
            return
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        return

    try:
        start = rospy.ServiceProxy('/dji_sdk/mission_waypoint_action', MissionWpAction)
        resp = start(0)
        rospy.loginfo('Service. Mission waypoint action:')
        rospy.loginfo(resp.result)
        if not resp.result:
            return
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        return


class FlightMission:
    def __init__(self):
        rospy.init_node('de_airsense_mission')

        self.status_in_air = False

        rospy.loginfo('Waiting for dji_sdk services...')
        rospy.wait_for_service('/dji_sdk/sdk_control_authority')
        rospy.wait_for_service('/dji_sdk/mission_waypoint_action')
        rospy.wait_for_service('/dji_sdk/mission_waypoint_upload')
        rospy.loginfo('Services are ok')

        self.__gpsPublisher = rospy.Publisher('de/drone/gps_position', NavSatFix, queue_size=100)
        self.__imuPublisher = rospy.Publisher('de/drone/imu', Imu, queue_size=100)
        self.__flightStarted = rospy.Publisher('de/drone/flight_started', Empty, queue_size=100)
        self.__flightEnded = rospy.Publisher('de/drone/flight_ended', Empty, queue_size=100)

        rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, self.gps_cb)
        rospy.Subscriber('/dji_sdk/imu', Imu, self.imu_cb)
        rospy.Subscriber('/dji_sdk/flight_status', UInt8, self.flight_status_cb)

        rospy.Subscriber('de/drone/mission', Mission, new_mission_cb)
        rospy.loginfo('Waiting for objective at "de/drone/mission" topic ...')

    def gps_cb(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        rospy.loginfo_throttle(1, 'position: %s, %s' % (self.lat, self.lon))
        self.__gpsPublisher.publish(data)

    def imu_cb(self, data):
        self.__imuPublisher.publish(data)

    def flight_status_cb(self, data_msg):
        if data_msg.data == IN_AIR_STANDBY and self.status_in_air is False:
            self.status_in_air = True
            self.__flightStarted.publish()
            rospy.loginfo('mission started')
        elif data_msg.data == ON_GROUND and self.status_in_air is True:
            self.status_in_air = False
            self.__flightEnded.publish()
            rospy.loginfo('mission ended')

    def spin(self):
        rospy.spin()
