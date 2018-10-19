# -*- coding: utf-8 -*-

from robonomics_lighthouse.msg import Ask, Bid
from std_msgs.msg import String
from std_srvs.srv import Empty
import rospy
from web3 import Web3, HTTPProvider
from threading import Thread


class Agent:

    current_measurement = None
    # in_work = False

    def __init__(self):
        rospy.init_node('de_airsense_agent')

        self.model = rospy.get_param('~model')
        self.token = rospy.get_param('~token')
        self.bid_lifetime = rospy.get_param('~bid_lifetime')
        self.web3 = Web3(HTTPProvider(rospy.get_param('~web3_http_provider')))
        self.signing_bid_pub = rospy.Publisher('liability/infochan/signing/bid', Bid, queue_size=10)

        def incoming_ask(ask_msg):
            rospy.loginfo('Incoming ask: ' + str(ask_msg))
            if ask_msg.model == self.model and ask_msg.token == self.token:
            	# self.in_work = True
                rospy.loginfo('Incoming ask with right model and token.')
                self.make_bid(ask_msg)
            else:
                rospy.loginfo('Incoming ask with wrong model and token, skip.')
        rospy.Subscriber('liability/infochan/incoming/ask', Ask, incoming_ask)

        def measurements(hash_msg):
            self.current_measurement = hash_msg.data
            rospy.loginfo('Received measurements in IPFS file: ' + hash_msg.data)
        rospy.Subscriber('de_airsense_waspmote_ipfs/result/measurements', String, measurements)

        rospy.wait_for_service('liability/finish')
        self.finish_srv = rospy.ServiceProxy('liability/finish', Empty)

        Thread(target=self.process, daemon=True).start()

    def make_bid(self, incoming_ask):
        rospy.loginfo('Making bid...')

        bid = Bid()
        bid.model = self.model
        bid.objective = incoming_ask.objective
        bid.token = self.token
        bid.cost = incoming_ask.cost
        bid.lighthouseFee = 0
        bid.deadline = self.web3.eth.getBlock('latest').number + self.bid_lifetime
        self.signing_bid_pub.publish(bid)

    def process(self): # to subscriber cb?
        while True:
            while not self.current_measurement: # and not self.in_work
                rospy.sleep(1)
            self.current_measurement = None
            self.finish_srv()
            # self.in_work = False
            rospy.loginfo('Liability finished')
    
    def spin(self):
        rospy.spin()
