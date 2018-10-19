# -*- coding: utf-8 -*-

from . import agent 
from . import mission
from . import waspmote_ipfs

def agent_node():
    agent.Agent().spin()

def mission_node():
    mission.FlightMission().spin()

def waspmote_ipfs_node():
    waspmote_ipfs.WaspmoteIPFS().spin()

