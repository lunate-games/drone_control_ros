# -*- coding: utf-8 -*-

# import agent
# import waspmote_ipfs
import mission_px4, mission_dji


#def agent_node():
#    agent.Agent().spin()

def mission_dji_node():
    mission_dji.FlightMission().spin()

def mission_px4_node():
    mission_px4.FlightMission().spin()

# def waspmote_ipfs_node():
#     waspmote_ipfs.WaspmoteIPFS().spin()

