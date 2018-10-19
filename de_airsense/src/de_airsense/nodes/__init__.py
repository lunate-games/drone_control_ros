# -*- coding: utf-8 -*-
import mission_px4, mission_dji


def mission_dji_node():
    mission_dji.FlightMission().spin()


def mission_px4_node():
    mission_px4.FlightMission().spin()
