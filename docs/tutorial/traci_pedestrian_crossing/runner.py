#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

"""
Tutorial for traffic light control via the TraCI interface.
This scenario models a pedestrian crossing which switches on demand.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random


# the directory in which this script resides
THISDIR = os.path.dirname(__file__)


# we need to import python modules from the $SUMO_HOME/tools directory
# If the the environment variable SUMO_HOME is not set, try to locate the python
# modules relative to this script
try:
    # tutorial in tests
    sys.path.append(os.path.join(THISDIR, '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        THISDIR, "..", "..", "..")), "tools"))  # tutorial in docs

    import traci
    from sumolib import checkBinary  # noqa
    import randomTrips
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

class Car_Info:
    def __init__(self,id):
        self.id = id
        self.lasttime = 0
        self.n = {}
        self.St = {}
        self.Ut = {}
class Link_Info:
    def __init__(self,id):
        self.id = id
        self.lasttime = 0
        self.outnum = []
        self.outcount = -1
        self.outflow = 0
        self.innum = []
        self.incount = 0
        self.lastvehs = [0]


"""V2V update: do V2V communication among drivers in the same link."""
# # Problem: How to calulate set
# getDistance2D(self, x1, y1, x2, y2, isGeo=False, isDriving=False)
def V2Vupdate(carid, range, car_list, netcars, linklist, linkidlist):
    tempcar = car_list[carid]
    tempcarPos = traci.vehicle.getPosition(carid)
    temp_group = []
    temp_group.append(tempcar)
    for i in netcars:
        ipos = traci.vehicle.getPosition(i)
        Distance = traci.simulation.getDistance2D(tempcarPos[0], tempcarPos[1], ipos[0], ipos[1], 0, 0)
        if ((Distance > 0) and (Distance <= range)):
            temp_group.append(car_list[i])
    if (len(temp_group) >= 2):
        for j in linkidlist:
            n = []
            st = []
            ut = []
            for k in temp_group:
                if j in k.n.keys():
                    n = n + k.n[j]
                    st = st + k.St[j]
                    ut = ut + k.Ut[j]
            nset = set(n)
            stset = set(st)
            utset = set(ut)
            nlist = list(nset)
            stlist = list(stset)
            utlist = list(utset)
            for h in temp_group:
                h.n[j] = nlist
                h.St[j] = stlist
                h.Ut[j] = utlist

"""Information update when CVs exit the link"""
# distance between two vehicles :(traci.simulationdomain.getDistanceRoad(self, edgeID1, pos1, edgeID2, pos2, isDriving=False))  Distance Type==1: driving distance
# or getDrivingDistance(self, vehID, edgeID, pos, laneIndex=0)
# vehicle speed: traci.vehicle.getSpeed(id)
# edgeID:
# edge position: traci.vehicle.getLanePosition(leadvehid)
# h(t) = h(s) / vehicle speed. (timeheadway =spaceheadway/speed)
# flow = 1/timeheadway
# qin = maximam incoming flow = minimum gap it is fixed value
# Exit information V2Vupdate
def ExitUpdate(linklist, car_list, linkid, time):
    if ((traci.edge.getLastStepHaltingNumber(linkid) > 0) and (traci.edge.getLastStepVehicleNumber(linkid) > 3)):
        n = traci.edge.getLastStepHaltingNumber(linkid)
        vehIDs = traci.edge.getLastStepVehicleIDs(linkid)
        leadvehid = vehIDs[-1]
        lead2Vehid = vehIDs[-2]
        lastvehid = vehIDs[0]
        # lastvehspeed =  traci.vehicle.getSpeed(lastvehid) #[m/s]
        # lead2Vehspeed = traci.vehicle.getSpeed(lead2Vehid)
        # InDistanceheadwayMin = traci.vehicle.getMinGap(lastvehid) #[m]
        # OutDistanceheadway = traci.vehicle.getDrivingDistance(lead2Vehid, linkid, traci.vehicle.getLanePosition(leadvehid), laneIndex=0) #[m]
        InTimeHeadwayMin = traci.vehicle.getTau(lastvehid) # [s]
        OutTimeheadway = traci.vehicle.getTau(lastvehid) #OutDistanceheadway / lead2Vehspeed #[s]
        Outflow = linklist[linkid].outflow #veh/s
        Inflowmax = 1/InTimeHeadwayMin #veh/s
        tempCar = car_list[leadvehid]
        if (linkid not in tempCar.n.keys() ):
            tempCar.n[linkid]= []
            tempCar.St[linkid] = []
            tempCar.Ut[linkid] = []
        tempCar.n[linkid].append((time, n))
        tempCar.St[linkid].append((time, -Outflow))
        tempCar.Ut[linkid].append((time,(Inflowmax - Outflow)))
        tempCar.lasttime = time
#
#
# minimum green time for the vehicles
MIN_GREEN_TIME = 15
# the first phase in tls plan. see 'pedcrossing.tll.xml'
VEHICLE_GREEN_PHASE = 0
# the id of the traffic light (there is only one). This is identical to the
# id of the controlled intersection (by default)
TLSID = 'C'

# pedestrian edges at the controlled intersection
WALKINGAREAS = [':C_w0', ':C_w1']
CROSSINGS = [':C_c0']


def run():
    """execute the TraCI control loop"""
    # track the duration for which the green phase of the vehicles has been
    # active
    greenTimeSoFar = 0
    # whether the pedestrian button has been pressed
    activeRequest = False
    # Link list
    Link_id_list = traci.edge.getIDList()
    Link_list = {}
    for i in Link_id_list:
        Link_list[i] = Link_Info(i)
    # Car_list
    Car_list = {}
    # main loop. do something every simulation step until no more vehicles are
    # loaded or running
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        for carid in traci.vehicle.getIDList():
            if carid not in Car_list.keys():
                Car_list[carid] = Car_Info(carid)
        # link inflow and outflow Update
        for j in traci.edge.getIDList():
            CurrentVehs = traci.edge.getLastStepVehicleIDs(j)
            temp_link = Link_list[j]
            if (CurrentVehs != []):
                if (CurrentVehs[-1] != temp_link.lastvehs[-1]):
                    temp_link.outcount += 1
                    temp_out = (traci.simulation.getCurrentTime()/1000,temp_link.outcount)
                    time = traci.simulation.getCurrentTime()/1000
                    temp_link.outflow = 1.0*1000 * temp_link.outcount / traci.simulation.getCurrentTime()
                    temp_link.outnum.append(temp_out)
                    temp_link.lastvehs = CurrentVehs
                    ExitUpdate(Link_list, Car_list,j,time)
        for h in traci.vehicle.getIDList():
            V2Vupdate(h, 5, Car_list, traci.vehicle.getIDList(), Link_list, Link_id_list)
        # print(traci.simulation.getCurrentTime()/1000)
        # print(Link_list["WC"].outnum)
        # decide wether there is a waiting pedestrian and switch if the green
        # phase for the vehicles exceeds its minimum duration
        if not activeRequest:
            activeRequest = checkWaitingPersons()
        if traci.trafficlight.getPhase(TLSID) == VEHICLE_GREEN_PHASE:
            greenTimeSoFar += 1
            if greenTimeSoFar > MIN_GREEN_TIME:
                # check whether someone has pushed the button

                if activeRequest:
                    # switch to the next phase
                    traci.trafficlight.setPhase(
                        TLSID, VEHICLE_GREEN_PHASE + 1)
                    # reset state
                    activeRequest = False
                    greenTimeSoFar = 0

    sys.stdout.flush()
    traci.close()


def checkWaitingPersons():
    """check whether a person has requested to cross the street"""

    # check both sides of the crossing
    for edge in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(edge)
        # check who is waiting at the crossing
        # we assume that pedestrians push the button upon
        # standing still for 1s
        for ped in peds:
            if (traci.person.getWaitingTime(ped) == 1 and
                    traci.person.getNextEdge(ped) in CROSSINGS):
                print("%s pushes the button" % ped)
                return True
    return False


def get_options():
    """define options for this script and interpret the command line"""
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    # load whether to run with or without GUI
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    net = 'pedcrossing.net.xml'
    # build the multi-modal network from plain xml inputs
    subprocess.call([checkBinary('netconvert'),
                     '-c', os.path.join('data', 'pedcrossing.netccfg'),
                     '--output-file', net],
                    stdout=sys.stdout, stderr=sys.stderr)

    # generate the pedestrians for this simulation
    randomTrips.main(randomTrips.get_options([
        '--net-file', net,
        '--output-trip-file', 'pedestrians.trip.xml',
        '--seed', '42',  # make runs reproducible
        '--pedestrians',
        '--prefix', 'ped',
        # prevent trips that start and end on the same edge
        '--min-distance', '1',
        '--trip-attributes', 'departPos="random" arrivalPos="random"',
        '--binomial', '4',
        '--period', '35']))

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, '-c', os.path.join('data', 'run.sumocfg')])
    run()
