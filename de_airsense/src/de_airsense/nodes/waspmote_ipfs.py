# -*- coding: utf-8 -*-

import serial
import time
import os
import errno
import ipfsapi
from std_msgs.msg import UInt8, Float32, String
from sensor_msgs.msg import NavSatFix
import rospy
from threading import Thread

class WaspmoteIPFS:

    status_in_air = False
    status_to_send = False
    altitude_rel = 0.0
    altitude_gps = 0.0
    latitude = 0.0
    longitude = 0.0

    def __init__(self):
        DEFAULT_PORT = '/dev/ttyUSB0'
        BAUDRATE = 115200
        IN_AIR_STANDBY = 3
        ON_GROUND = 1

        rospy.loginfo('Starting waspmote gas sensors...')
        rospy.loginfo('Waiting for ROS services...')
        rospy.init_node('de_airsense_waspmore_ipfs')

        def flight_status(data_msg): 
            if data_msg.data == IN_AIR_STANDBY and self.status_in_air == False:
                self.status_in_air = True
                rospy.loginfo('Start to write data')
            elif data_msg.data == ON_GROUND and self.status_in_air == True:
                rospy.loginfo('Stop to write data')
                self.status_to_send = True
                self.status_in_air = False
        rospy.Subscriber('dji_sdk/flight_status', UInt8, flight_status)

        def gps_position(data_msg):
            self.latitude = data_msg.latitude
            self.longitude = data_msg.longitude
            self.altitude_gps = data_msg.altitude
        rospy.Subscriber('dji_sdk/gps_position', NavSatFix, gps_position)

        def height_above_takeoff(data_msg):
            self.altitude_rel = data_msg.data
        rospy.Subscriber('dji_sdk/height_above_takeoff', Float32, height_above_takeoff)

        self.result_pub = rospy.Publisher('~result/measurements', String, queue_size=10)
        self.ipfs_api_loc = ipfsapi.connect('127.0.0.1', 5001)
        # ipfs_api_rem = ipfsapi.connect('52.178.98.62', 9095)

        self.serial_port = serial.Serial(   DEFAULT_PORT, 
                                            BAUDRATE, 
                                            parity=serial.PARITY_NONE, 
                                            stopbits=serial.STOPBITS_ONE, 
                                            bytesize=serial.EIGHTBITS)

        thread = Thread(target=self.serial_receiver, daemon=True).start()

    def serial_receiver(self):
        STOP_SYMBOL = b'$'
        rate = rospy.Rate(1)
        frame = ''
        frame_array = []
        waspmote_ready = False
        while not rospy.is_shutdown():
            try:    
                while self.serial_port.in_waiting > 0:
                    if waspmote_ready == False:
                        waspmote_ready = True
                        rospy.loginfo('Waspmote gas sensors and ROS are ready')
                    byte = self.serial_port.read()

                    if byte == STOP_SYMBOL:
                        # frame += byte.decode()  
                        frame +=    'Copter Latitude: {0:.6f}\n' \
                                    'Copter Longitude: {1:.6f}\n' \
                                    'Copter GPS Altitude: {2:.2f} m\n' \
                                    'Copter Relative Altitude: {3:.2f} m\n' \
                                    'System time: {4:s}\n'.format(  self.latitude, 
                                                                    self.longitude, 
                                                                    self.altitude_gps,
                                                                    self.altitude_rel, 
                                                                    time.strftime('%Y/%m/%d %H:%M:%S'))
                        frame_array.append(frame)
                        rospy.loginfo(frame)
                        frame = ''
                        continue
                    elif byte != b'\x86' and byte != b'\x00':
                        frame += byte.decode()

                if self.status_to_send:
                    self.status_to_send = False
                    self.write_send_data(frame_array)
                    frame_array = []
                elif self.status_in_air == False:
                    frame_array = []

                rate.sleep()

            except KeyboardInterrupt: 
                rospy.loginfo('\nExit')
                break

    def write_send_data(self, frame_array):
        fileName = 'data_'
        dir = os.path.dirname(__file__)
        folderName = os.path.join(dir, 'sensor_data/')
        if not os.path.exists(os.path.dirname(folderName)):
            try:
                os.makedirs(os.path.dirname(folderName))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
                    
        timestr = time.strftime('%Y-%m-%d_%H-%M-%S')
        path = folderName + fileName + timestr + '.txt'
        with open (path, 'w') as f:
            for string in frame_array:
                f.write(string)
        rospy.loginfo('File wrote in')
        rospy.loginfo(path)

        def ipfs_send (api, file):
            try:
                result = api.add(file)
                return result
            except:
                return False

        result = ipfs_send(self.ipfs_api_loc, path)
        rospy.logdebug('IPFS local node response:')
        rospy.logdebug(result)
        if result['Hash']:
            rospy.loginfo('IPFS file hash:')
            rospy.loginfo(result['Hash'])
            result_msg = String()
            result_msg.data = result['Hash']
            self.result_pub.publish(result_msg)
        else:
            rospy.loginfo('IPFS error')

    def spin(self):
        rospy.spin()