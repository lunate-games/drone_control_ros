# -*- coding: utf-8 -*-

import rospy
from dji_sdk.srv import SDKControlAuthority, MissionWpUpload, MissionWpAction
from dji_sdk.msg import MissionWaypointAction, MissionWaypointTask, MissionWaypoint
from de_msgs.msg import Mission
from sensor_msgs.msg import NavSatFix, Imu


def mission_start(mission_msg):
    try:
        auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
        rospy.loginfo('Service. Sdk control authority:')
        resp = auth(1)
        rospy.loginfo(resp.result)
        if not resp.result:
            return
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        return

    try:
        mission = rospy.ServiceProxy('dji_sdk/mission_waypoint_upload', MissionWpUpload)
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
        start = rospy.ServiceProxy('dji_sdk/mission_waypoint_action', MissionWpAction)
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

        rospy.loginfo('Waiting for dji_sdk services...')
        rospy.wait_for_service('dji_sdk/sdk_control_authority')
        rospy.wait_for_service('dji_sdk/mission_waypoint_action')
        rospy.wait_for_service('dji_sdk/mission_waypoint_upload')
        rospy.loginfo('Services are ok')

        self.__gpsPublisher = rospy.Publisher('de/drone/gps_position', NavSatFix, queue_size=100)
        self.__imuPublisher = rospy.Publisher('de/drone/imu', Imu, queue_size=100)

        def gps_cb(data):
            self.lat = data.latitude
            self.lon = data.longitude
            rospy.loginfo_throttle(1, 'position: %s, %s' % (self.lat, self.lon))
            self.__gpsPublisher.publish(data)

        rospy.Subscriber('dji_sdk/gps_position', NavSatFix, gps_cb)

        def imu_cb(data):
            self.__imuPublisher.publish(data)

        rospy.Subscriber("/dji_sdk/imu", Imu, imu_cb)

        rospy.Subscriber('de/drone/mission', Mission, mission_start)
        rospy.loginfo('Waiting for objective at "de/drone/mission" topic ...')

    def spin(self):
        rospy.spin()
