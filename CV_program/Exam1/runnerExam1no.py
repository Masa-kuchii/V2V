"""
Program used in the article "Queue length estimation through a simple V2V communication protocol"
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import csv
from numpy.random import *

# the directory where this script resides
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
    def __init__(self,id,CV_bool):
        self.id = id
        self.lasttime = 0
        self.n = {}
        self.St = {}
        self.Ut = {}
        self.Stvalue = {} # linkid:(StValue)
        self.Utvalue = {} # linkid:(UtValue)
        self.StslopeValue = {}
        self.UtslopeValue = {}
        self.CVlabel = CV_bool
        self.LinkExperi = {}
    def GetStUtValue(self,linkid):
        st = self.Stvalue[linkid]
        ut = self.Utvalue[linkid]
        return (st, ut)
    def GetStUtSlope(self,linkid):
        stslope = self.StslopeValue[linkid]
        utslope = self.UtslopeValue[linkid]
        return (stslope, utslope)

class Link_Info:
    def __init__(self,id):
        self.id = id
        self.lasttime = 0
        self.outnum = []
        self.outcount = -1
        self.outflow = 0
        self.inflow = GetEdgeCapacity(id)
        self.outflowhead = 0
        self.innum = []
        self.incount = 0
        self.lastvehs = []
        self.queue = 0
        self.Loss = 0
        self.Losshis = {0:0}
    def GetLinkcapacity(self):
        return self.inflow

class Node_Info:
    def __init__(self, id):
        self.id = id
        self.position = traci.junction.getPosition(id)
        self.tempcomgroup = []
        self.linkSt = {}
        self.linkUt = {}
        self.linkStSlope = {}
        self.linkUtSlope = {}
        self.past = {}
    def groupClear(self):
        self.tempcomgroup = []
        return 0
    def GetPosition(self):
        return self.position
    def PutCarIntoGroup(self, carclass):
        self.tempcomgroup.append(carclass)
    def GetGroup(self):
        return self.tempcomgroup
    def GetStUtValue(self,linkid):
        st = self.linkSt[linkid]
        ut = self.linkUt[linkid]
        return (st, ut)

class Eta:
    def __init__(self):
        self.value = 1.0
        self.hist = [(0,1.0)]
        self.dic = {0:1.0}
        self.Lossdic = [(0,1.0)]
        self.Lossnumegrad = [(0,1.0)]
        self.Losssimpgrad = [(0,1.0)]
    def GetEta(self):
        return self.value
    def CountInconsis(self, carids, carlist, QueueLinkIDs,time):
        num_con = 0
        num_incon = 0
        for linkid in QueueLinkIDs:
            InconPattern = []
            for j in carids:
                car = carlist[j]
                St = 0.0
                Ut = 100000
                CarDic ={}
                if (linkid in car.n.keys()):
                    for n in car.n[linkid]:
                        Car_n_Time = n[0]
                        Car_n_Value = n[1]
                        CarDic[Car_n_Time] = Car_n_Value
                    for st in car.St[linkid]:
                        Car_st_Time = st[0]
                        Car_st_Value = st[1]
                        sum_st_slope = 0
                        for tichan in range(Car_st_Time,time,1):
                            sum_st_slope += self.dic[tichan]*Car_st_Value*1.0
                        newst = CarDic[Car_st_Time] + sum_st_slope
                        if (newst > St):
                            St = CarDic[Car_st_Time] + sum_st_slope
                    for ut in car.Ut[linkid]:
                        Car_ut_Time = ut[0]
                        Car_ut_Value = ut[1]
                        sum_ut_slope = 0
                        for tichan in range(Car_ut_Time,time,1):
                            sum_ut_slope += self.dic[tichan]*Car_ut_Value*1.0
                        newut = CarDic[Car_ut_Time] + sum_ut_slope
                        if (newut < Ut):
                            Ut = CarDic[Car_ut_Time] + sum_ut_slope
                    if (St > Ut):
                        Stpattern = (Car_st_Time, Car_st_Value)
                        Utpattern = (Car_ut_Time, Car_ut_Value)
                        if ((Stpattern not in InconPattern) and (Utpattern not in InconPattern)):
                            num_incon += 1
                            InconPattern.append(Stpattern)
                            InconPattern.append(Utpattern)
                    else:
                        num_con += 1
        return num_incon # does not use currently
    def FixedUpdate(self, carids, carlist, QueueLinkIDs, time, time_interval):
        num_incon = self.CountInconsis(carids, carlist, QueueLinkIDs,time)
        current_eta = self.GetEta()
        if (num_incon == 0):
            next_eta = current_eta * 0.95 #5% down
        else:
            next_eta = current_eta * pow(1.2, num_incon) #20% up
        self.value = next_eta
        self.hist.append((time, next_eta))
        self.dic[time] = next_eta
        return 0 # does not use currently
    def Lossgradient(self, carids, carlist, QueueLinkIDs,linklist,time,whichLoss = True):
        h = 1e-4
        Loss = 0
        Lossalphatasu = 0
        Lossalphahiku = 0
        for linkid in QueueLinkIDs:
            link_temp = linklist[linkid]
            link_temp.Loss = 0
            for j in carids:
                car = carlist[j]
                St = 0.0
                Ut = 100000
                CarDic ={}
                if (car.CVlabel == 1):
                    if (linkid in car.n.keys()):
                        if (len(car.n[linkid])>=1):
                            StAndUtValue = car.GetStUtValue(linkid)
                            StAndUtSlope = car.GetStUtSlope(linkid)
                            St = StAndUtValue[0]
                            Ut = StAndUtValue[1]
                            ture_st_slope = StAndUtSlope[0]
                            ture_ut_slope = StAndUtSlope[1]
                            # Calculate using simple Loss LossFunction (kuchii's definition)
                            if (whichLoss == True):
                                Loss += self.LossFunction(self.value, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                                Lossalphatasu += self.LossFunction(self.value+h, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                                Lossalphahiku += self.LossFunction(self.value-h, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                            # Calculate Loss function Gunnar's definition
                            elif (whichLoss == False):
                                Loss += self.SimpleLossFunction(self.value, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                                Lossalphatasu += self.SimpleLossFunction(self.value+h, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                                Lossalphahiku += self.SimpleLossFunction(self.value-h, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
        self.Lossdic.append((time,Loss))
        gradient = (Lossalphatasu - Lossalphahiku)/2/h
        self.Lossnumegrad.append((time,gradient))
        return gradient
    def LossFunction(self, neweta, oldst, stslope, oldut, utslope, ntrue):
        newst = oldst - self.value*stslope*1.0 + neweta*stslope*1.0
        newut = oldut - self.value*utslope*1.0 + neweta*utslope*1.0
        U = (newut-newst)*(newut-newst)/2
        E = (ntrue - (newut+newst)/2)*(ntrue - (newut+newst)/2)/2
        return  (U + E)
    def SimpleLossFunction(self, neweta, oldst, stslope, oldut, utslope, ntrue):
        newst = oldst - self.value*stslope*1.0 + neweta*stslope*1.0
        newut = oldut - self.value*utslope*1.0 + neweta*utslope*1.0
        numerator = (ntrue-(newst+newut)/2)
        denominator = max((newut-newst), 2.0)
        U = numerator*numerator/denominator/denominator/2
        return  (U)
    def LossFunction_directGrad(self, carids, carlist, QueueLinkIDs,linklist,time):
        h = 1e-4
        Loss = 0
        Loss_gradient = 0
        for linkid in QueueLinkIDs:
            link_temp = linklist[linkid]
            link_temp.Loss = 0
            for j in carids:
                C1 = 0
                C2 = 0
                C3 = 0
                C4 = 0
                car = carlist[j]
                St = 0.0
                Ut = 100000
                CarDic ={}
                if (linkid in car.n.keys()):
                    ture_st_slope = 100
                    ture_ut_slope = 100
                    for n in car.n[linkid]:
                        Car_n_Time = n[0]
                        Car_n_Value = n[1]
                        CarDic[Car_n_Time] = Car_n_Value
                    for st in car.St[linkid]:
                        Car_st_Time = st[0]
                        Car_st_Value = st[1]
                        if (ture_st_slope == 100):
                            ture_st_slope = Car_st_Value
                        sum_st_slope = 0
                        for tichan in range(Car_st_Time,time,1):
                            sum_st_slope += self.dic[tichan]*Car_st_Value*1.0
                        newst = CarDic[Car_st_Time] + sum_st_slope + Car_st_Value*self.value
                        if (newst > St):
                            St = CarDic[Car_st_Time] + sum_st_slope + Car_st_Value*self.value
                            ture_st_slope = Car_st_Value
                            ture_st_n = CarDic[Car_st_Time]
                    for ut in car.Ut[linkid]:
                        Car_ut_Time = ut[0]
                        Car_ut_Value = ut[1]
                        if (ture_ut_slope == 100):
                            ture_ut_slope = Car_ut_Value
                        sum_ut_slope = 0
                        for tichan in range(Car_ut_Time,time,1):
                            sum_ut_slope += self.dic[tichan]*Car_ut_Value*1.0
                        newut = CarDic[Car_ut_Time] + sum_ut_slope + Car_ut_Value*self.value
                        if (newut < Ut):
                            Ut = CarDic[Car_ut_Time] + sum_ut_slope + Car_ut_Value*self.value
                            ture_ut_slope = Car_ut_Value
                            ture_ut_n = CarDic[Car_ut_Time]
                    # Calculate Loss function
                    Loss += self.LossFunction(self.value, St, ture_st_slope,Ut,ture_ut_slope,link_temp.queue)
                    C1 = link_temp.queue - ((Ut- self.value * ture_ut_slope)+(St - self.value * ture_st_slope))/2
                    C2 = -(ture_ut_slope + ture_st_slope)/2
                    C3 = (ture_ut_slope - ture_st_slope)
                    C4 = (Ut- self.value * ture_ut_slope) - (St - self.value * ture_st_slope)
                    Loss_gradient += C3*(Ut-St) + C2*(link_temp.queue-(Ut+St)/2)
        # self.Lossdic.append((time,Loss))
        self.Losssimpgrad.append((time,Loss_gradient))
        return Loss_gradient
    def LossUpdate(self,carids, carlist, QueueLinkIDs,linklist,time, step_size, Grad = True):
        if (Grad == True):
            dldeta = self.LossFunction_directGrad(carids, carlist, QueueLinkIDs,linklist, time)
        elif (Grad == False):
            dldeta = self.Lossgradient(carids, carlist, QueueLinkIDs,linklist, time, False)
        current_eta = self.GetEta()
        next_eta = current_eta - dldeta*step_size
        self.value = next_eta
        self.hist.append((time, current_eta))
        self.dic[time+1] = next_eta
        return 0
    def Oneupdate(self, time):
        current_eta = self.GetEta()
        self.hist.append((time,current_eta))
        self.Lossdic.append((time,0))
        self.Lossnumegrad.append((time,0))
        self.dic[time+1] = current_eta
        return 0
    def csvoutput(self, writer):
        list = self.hist
        loss_list = self.Lossdic
        GradNume = self.Lossnumegrad
        # Gradmath = self.Losssimpgrad
        header = ["time", "etavalue", "Loss", "Grad"]
        writer.writerow(header)
        for i in range(len(list)):
            time = list[i][0]
            value = list[i][1]
            loss = loss_list[i][1]
            gradnu = GradNume[i][1]
            temp = [time, value, loss, gradnu]
            writer.writerow(temp)



"""V2V update: V2V communication among drivers in the same link."""
def V2Vupdate(carid, comrange, car_list, netcars, linklist, linkidlist,time, eta):
    tempcar = car_list[carid]
    tempcarPos = traci.vehicle.getPosition(carid)
    temp_group = []
    temp_group.append(tempcar)
    for i in netcars:
        icarclass = car_list[i]
        if (icarclass.CVlabel == 1):
            ipos = traci.vehicle.getPosition(i)
            Distance = traci.simulation.getDistance2D(tempcarPos[0], tempcarPos[1], ipos[0], ipos[1], 0, 0)
            if ((Distance > 0) and (Distance <= comrange)):
                temp_group.append(car_list[i])
    if (len(temp_group) >= 2):
        for j in linkidlist:
            n = []
            st = []
            ut = []
            link_vehs = traci.edge.getLastStepVehicleIDs(j)
            for k in temp_group:
                if j in k.n.keys():
                    if (link_vehs != []):
                        if ( k.id == link_vehs[-1] and traci.vehicle.getSpeed(k.id) <= 1.0 and time >= 500):
                            n.append((time, GetQueueLengthCV(k.id, car_list, j, comrange)))
                            st.append((time, -(linklist[j].outflow)))
                            ut.append((time, (linklist[j].inflow - linklist[j].outflow)))
                    n = n + k.n[j]
                    st = st + k.St[j]
                    ut = ut + k.Ut[j]
            nset = set(n)
            stset = set(st)
            utset = set(ut)
            nlist = list(nset)
            nlist_small = [i for i in nlist if (time - i[0]) < 300]
            stlist = list(stset)
            stlist_small = [i for i in stlist if (time - i[0]) < 300]
            utlist = list(utset)
            utlist_small = [i for i in utlist if (time - i[0]) < 300]
            if (nlist != []):
                for h in temp_group:
                    h.n[j] = nlist_small
                    h.St[j] = stlist_small
                    h.Ut[j] = utlist_small


"""Information update when CVs exit the link"""
def ExitUpdate(linklist, car_list, linkid, time, outputcarid, comrange, eta):
    if ((outputcarid != 0) and (traci.edge.getLastStepVehicleNumber(linkid) > 3) and outputcarid in traci.vehicle.getIDList()):
        n = GetQueueLengthCV(outputcarid, car_list, linkid, comrange)
        if (n > 0):
            Outflow = linklist[linkid].outflow #veh/s
            Inflowmax = 0.55 #veh/s here, GetlinkCapacity
            tempCar = car_list[outputcarid]
            if (linkid not in tempCar.n.keys() ):
                tempCar.n[linkid]= []
                tempCar.St[linkid] = []
                tempCar.Ut[linkid] = []
            tempCar.n[linkid].append((time, n))
            tempCar.St[linkid].append((time, -Outflow))
            tempCar.Ut[linkid].append((time,(Inflowmax - Outflow)))
            tempCar.lasttime = time
    return 0

def GetQueueLengthCV(carid, car_list, linkid, comrange):
    car = car_list[carid]
    carPos = traci.vehicle.getPosition(carid)
    vehIDs = traci.edge.getLastStepVehicleIDs(linkid)
    vehIDs_reverse = []
    for i in vehIDs:
        CVcarornot = car_list[i]
        if (CVcarornot.CVlabel == 1):
            vehIDs_reverse.append(i)
    vehIDs_reverse.reverse()
    queue = 0
    platoon = []
    for i in range(len(vehIDs_reverse)):
        mae = vehIDs_reverse[i]
        mae_posi = traci.vehicle.getPosition(mae)
        if (traci.simulation.getDistance2D(carPos[0], carPos[1], mae_posi[0], mae_posi[1], 0, 0) > comrange or mae == vehIDs_reverse[-1]):
            break
        ushiro = vehIDs_reverse[i+1]
        ushiro_posi = traci.vehicle.getPosition(ushiro)
        if (i == 0 and traci.simulation.getDistance2D(carPos[0], carPos[1], ushiro_posi[0], ushiro_posi[1], 0, 0) > comrange):
            if (traci.vehicle.getSpeed(mae) <= 5.0):
                queue = 1
                break
            else:
                break
        two_veh_dis = traci.simulation.getDistance2D(mae_posi[0], mae_posi[1], ushiro_posi[0], ushiro_posi[1], 0, 0)
        if (two_veh_dis <= 15):
            if (mae not in platoon):
                platoon.append(mae)
            if (ushiro not in platoon):
                platoon.append(ushiro)
    if (platoon != []):
        for j in platoon:
            if (traci.vehicle.getSpeed(j) <= 5.0):
                queue += 1
    return queue

def GetQueueLengthTrue(carid, car_list, linkid, comrange):
    car = car_list[carid]
    carPos = traci.vehicle.getPosition(carid)
    vehIDs = traci.edge.getLastStepVehicleIDs(linkid)
    vehIDs_reverse = traci.edge.getLastStepVehicleIDs(linkid)
    vehIDs_reverse.reverse()
    queue = 0
    platoon = []
    for i in range(len(vehIDs)):
        mae = vehIDs_reverse[i]
        mae_posi = traci.vehicle.getPosition(mae)
        if (traci.simulation.getDistance2D(carPos[0], carPos[1], mae_posi[0], mae_posi[1], 0, 0) > comrange or mae == vehIDs_reverse[-1]):
            break
        ushiro = vehIDs_reverse[i+1]
        ushiro_posi = traci.vehicle.getPosition(ushiro)
        if (i == 0 and traci.simulation.getDistance2D(carPos[0], carPos[1], ushiro_posi[0], ushiro_posi[1], 0, 0) > comrange):
            if (traci.vehicle.getSpeed(mae) <= 5.0):
                queue = 1
                break
            else:
                break
        two_veh_dis = traci.simulation.getDistance2D(mae_posi[0], mae_posi[1], ushiro_posi[0], ushiro_posi[1], 0, 0)
        if (two_veh_dis <= 15):
            if (mae not in platoon):
                platoon.append(mae)
            if (ushiro not in platoon):
                platoon.append(ushiro)
    if (platoon != []):
        for j in platoon:
            if (traci.vehicle.getSpeed(j) <= 5.0):
                queue += 1
    return queue


def GetEdgeCapacity(linkid):
    inflowmaxcurrent = 0.55
    try:
        lanenum = traci.edge.getLaneNumber(linkid)
    except AttributeError:
        lanenum = 1
    capacity = inflowmaxcurrent*lanenum
    return capacity


def NodeEstimation(Nodelist,VechilesOnNetwork,comrange,eta,carlist,linkidlist, time):
    # add CVs within each node communication range
    for carid in VechilesOnNetwork:
        car = carlist[carid]
        if (car.CVlabel == 1):
            carposition = traci.vehicle.getPosition(carid)
            for nodeid, nodeclass in Nodelist.items():
                nodeposition = nodeclass.GetPosition()
                two_distance = traci.simulation.getDistance2D(carposition[0], carposition[1], nodeposition[0], nodeposition[1], 0, 0)
                if (two_distance <= comrange):
                    nodeclass.PutCarIntoGroup(car)
    # Calculate upper and lower bounds at each node
    for linkid in linkidlist:
        for nodeid2, nodeclass2 in Nodelist.items():
            St = []
            Ut = []
            StSlope = {}
            UtSlope = {}
            nodeVehGroup = nodeclass2.GetGroup()
            if (len(nodeVehGroup) != 0):
                for car in nodeclass2.GetGroup():
                    if (car.LinkExperi[linkid] == 1):
                        StAndUt = car.GetStUtValue(linkid)
                        carst = StAndUt[0]
                        carut = StAndUt[1]
                        St.append(carst)
                        Ut.append(carut)
                        StAndUtSlope = car.GetStUtSlope(linkid)
                        carstslope = StAndUtSlope[0]
                        carutslope = StAndUtSlope[1]
                        StSlope[carst] = carstslope
                        UtSlope[carut] = carutslope
                        DecidedSt = max(St)
                        DecidedUt = min(Ut)
                        nodeclass2.linkSt[linkid] = DecidedSt
                        nodeclass2.linkUt[linkid] = DecidedUt
                        nodeclass2.linkStSlope[linkid] = StSlope[DecidedSt]
                        nodeclass2.linkUtSlope[linkid] = UtSlope[DecidedUt]
                        nodeclass2.past[linkid] = 1
            else:
                if (linkid in nodeclass2.past.keys()):
                    prest = nodeclass2.linkSt[linkid]
                    preut = nodeclass2.linkUt[linkid]
                    nodeclass2.linkSt[linkid] = prest + eta.value * nodeclass2.linkStSlope[linkid]
                    nodeclass2.linkUt[linkid] = preut + eta.value * nodeclass2.linkUtSlope[linkid]
                else:
                    nodeclass2.linkSt[linkid] = 0
                    nodeclass2.linkUt[linkid] = 0
            nodeclass2.groupClear()

def VehicleStUtCalculation(CarsOnNetwork,time,linkidlist,eta, carlist):
    for linkid in linkidlist:
        for carid in CarsOnNetwork:
            car = carlist[carid]
            if (car.CVlabel == 1):
                if (linkid in car.n.keys()):
                    if (len(car.n[linkid])>=1):
                        St = 0.0
                        Ut = 100000
                        CarDic ={}
                        ture_st_slope = 100
                        ture_ut_slope = 100
                        for n in car.n[linkid]:
                            Car_n_Time = n[0]
                            Car_n_Value = n[1]
                            CarDic[Car_n_Time] = Car_n_Value
                        for st in car.St[linkid]:
                            Car_st_Time = st[0]
                            Car_st_Value = st[1]
                            if (ture_st_slope == 100):
                                ture_st_slope = Car_st_Value
                            sum_st_slope = 0
                            for tichan in range(Car_st_Time,time,1):
                                sum_st_slope += eta.dic[tichan]*Car_st_Value*1.0
                            newst = CarDic[Car_st_Time] + sum_st_slope + Car_st_Value*eta.value
                            if (newst > St):
                                St = CarDic[Car_st_Time] + sum_st_slope + Car_st_Value*eta.value
                                ture_st_slope = Car_st_Value
                                ture_st_n = CarDic[Car_st_Time]
                        for ut in car.Ut[linkid]:
                            Car_ut_Time = ut[0]
                            Car_ut_Value = ut[1]
                            if (ture_ut_slope == 100):
                                ture_ut_slope = Car_ut_Value
                            sum_ut_slope = 0
                            for tichan in range(Car_ut_Time,time,1):
                                sum_ut_slope += eta.dic[tichan]*Car_ut_Value*1.0
                            newut = CarDic[Car_ut_Time] + sum_ut_slope + Car_ut_Value*eta.value
                            if (newut < Ut):
                                Ut = CarDic[Car_ut_Time] + sum_ut_slope + Car_ut_Value*eta.value
                                ture_ut_slope = Car_ut_Value
                                ture_ut_n = CarDic[Car_ut_Time]
                        car.Stvalue[linkid] = St
                        car.Utvalue[linkid] = Ut
                        car.StslopeValue[linkid] = ture_st_slope
                        car.UtslopeValue[linkid] = ture_ut_slope
                        car.LinkExperi[linkid] = 1
                    else:
                        car.Stvalue[linkid] = 0
                        car.Utvalue[linkid] = 0
                        car.StslopeValue[linkid] = 0
                        car.UtslopeValue[linkid] = 0
                        car.LinkExperi[linkid] = 0
                else:
                    car.Stvalue[linkid] = 0
                    car.Utvalue[linkid] = 0
                    car.StslopeValue[linkid] = 0
                    car.UtslopeValue[linkid] = 0
                    car.LinkExperi[linkid] = 0


def Csvoutput(writer,time, linklist, car_list,comrange,eta):
    onlyidlist = ["AtoB","BtoA","AtoC","BtoC","CtoCright"]
    if (time == 1):
        name = ["time","queue length n","outflow","inflow","AtoB Lead st","AtoB Lead ut","AtoB last st","AtoB last ut","AtoBTT","AtoBEF","AtoBSpeed","BtoA Lead st","BtoA Lead ut","BtoA last st","BtoA last ut","BtoATT","BtoAEF","BtoASpeed","AtoC Lead st","AtoC Lead ut","AtoC last st","AtoC last ut","AtoCTT","AtoCEF","AtoCSpeed","BtoC Lead st","BtoC Lead ut","BtoC last st","BtoC last ut","BtoCTT","BtoCEF","BtoCSpeed","CtoCright Lead st","CtoCright Lead ut","CtoCright last st","CtoCright last ut","CtoCrightTT","CtoCrightEF","CtoCrightSpeed"]
        writer.writerow(name)
    else:
        val = []
        val.append(time)
        vehicles = traci.edge.getLastStepVehicleIDs("BtoA")
        n = 0
        if (len(vehicles) != 0):
            n = GetQueueLengthCV(vehicles[-1], car_list, "BtoA", comrange)
        val.append(n)
        outflowcal = 1.0*1000 * linklist["BtoA"].outcount / traci.simulation.getCurrentTime()
        val.append(outflowcal)
        val.append(0.55)
        for i in onlyidlist:
            if (traci.edge.getLastStepVehicleNumber(i) == 0):
                val.append(0)
                val.append(0)
                val.append(0)
                val.append(0)
                val.append(0)
                val.append(0)
                val.append(0)
            else:
                vehIDs = traci.edge.getLastStepVehicleIDs(i)
                leadvehid = vehIDs[-1]
                leadveh = car_list[leadvehid]
                laneid = i + "_0"
                if (leadveh.CVlabel == 1):
                    lestandut = leadveh.GetStUtValue("BtoA")
                    lestvalue = lestandut[0]
                    leutvalue = lestandut[1]
                else:
                    lestvalue = 0
                    leutvalue = 0
                val.append(lestvalue)
                val.append(leutvalue)
                lastvehid = vehIDs[0]
                lastveh = car_list[lastvehid]
                if (lastveh.CVlabel == 1):
                    lastandut = lastveh.GetStUtValue("BtoA")
                    lastvalue = lastandut[0]
                    lautvalue = lastandut[1]
                else:
                    lastvalue = 0
                    lautvalue = 0
                val.append(lastvalue)
                val.append(lautvalue)
                val.append(traci.edge.getTraveltime(i))
                flow = traci.edge.getLastStepVehicleNumber(i) / traci.lane.getLength(laneid) * traci.edge.getLastStepMeanSpeed(i)
                val.append(linklist[i].outflow)
                val.append(traci.edge.getLastStepMeanSpeed(i))
        writer.writerow(val)
    return 0

def Nodeoutput(writer, Nodeidlist, Nodelist, time, linkid):
    if (time == 1):
        header = ["time"]
        for nodeid in Nodeidlist:
            nodeS = nodeid + "_S"
            nodeU = nodeid + "_U"
            header.append(nodeS)
            header.append(nodeU)
        writer.writerow(header)
    else:
        row = []
        row.append(time)
        for nodeid in Nodeidlist:
            nodeclass = Nodelist[nodeid]
            St = 0
            Ut = 0
            if ((linkid in nodeclass.linkSt) and (linkid in nodeclass.linkUt)):
                StAndUt = nodeclass.GetStUtValue(linkid)
                St = StAndUt[0]
                Ut = StAndUt[1]
            row.append(St)
            row.append(Ut)
        writer.writerow(row)

def run():
    """execute the TraCI control loop"""
    # Link list
    Link_id_list = traci.edge.getIDList()
    Link_list = {}
    QueueLinkIDs = []
    for i in Link_id_list:
        Link_list[i] = Link_Info(i)
    # Node_list
    Node_id_list = traci.junction.getIDList()
    Node_list = {}
    for j in Node_id_list:
        Node_list[j] = Node_Info(j)
    # Car_list
    Car_list = {}
    # eta
    eta = Eta()
    # main loop. do something every simulation step until no more vehicles are
    # loaded or running
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        time = traci.simulation.getCurrentTime()/1000
        for carid in traci.vehicle.getIDList():
            if carid not in Car_list.keys():
                random_probability = rand()
                if (random_probability <= penetration_rate): # add label:1 to CV vehicles
                    Car_list[carid] = Car_Info(carid,1)
                elif (random_probability > penetration_rate): # add label:0 to non-CV vehicles
                    Car_list[carid] = Car_Info(carid, 0)
        # link inflow and outflow Update
        for j in traci.edge.getIDList():
            CurrentVehs = traci.edge.getLastStepVehicleIDs(j)
            temp_link = Link_list[j]
            if ((len(CurrentVehs) != 0)):
                if ((len(temp_link.lastvehs) == 0)):
                    temp_link.lastvehs = CurrentVehs
                temp_link.queue = GetQueueLengthTrue(CurrentVehs[-1], Car_list, "BtoA", communication_range)
                if (CurrentVehs[-1] != temp_link.lastvehs[-1]):
                    temp_link.outcount += 1
                    temp_out = (traci.simulation.getCurrentTime()/1000,temp_link.outcount)
                    headway = traci.simulation.getCurrentTime()/1000 - temp_link.lasttime
                    temp_link.outflow = 1.0*1000 * temp_link.outcount / traci.simulation.getCurrentTime()
                    # temp_link.outflowhead = 1.0 / headway
                    temp_link.outnum.append(temp_out)
                    outputCar = temp_link.lastvehs[-1]
                    outputCarclass = Car_list[outputCar]
                    temp_link.lasttime = traci.simulation.getCurrentTime()/1000
                    if (outputCarclass.CVlabel == 1):
                        ExitUpdate(Link_list, Car_list,j,time,outputCar,communication_range,eta)
                    temp_link.lastvehs = CurrentVehs
        for h in traci.vehicle.getIDList():
            v2vcarclass = Car_list[h]
            if (v2vcarclass.CVlabel == 1):
                V2Vupdate(h, communication_range, Car_list, traci.vehicle.getIDList(), Link_list, Link_id_list,time,eta)
        VehicleStUtCalculation(traci.vehicle.getIDList(),time,["BtoA"],eta,Car_list)  # Update St and Ut of all Vehicles
        # Update Eta
        eta.Oneupdate(time)
        # eta.FixedUpdate(traci.vehicle.getIDList(), Car_list, ["BtoA"], time, 1)
        # eta.LossUpdate(traci.vehicle.getIDList(), Car_list, ["BtoA"],Link_list,time, 0.01,False)
        # Update Node
        NodeEstimation(Node_list,traci.vehicle.getIDList(),communication_range,eta,Car_list,["BtoA"], time)
        # Output to CSV and Node
        Csvoutput(csvWriter,time, Link_list, Car_list, communication_range,eta)
        Nodeoutput(nodewriter, Node_id_list, Node_list, time, "BtoA")
        if (time == 4000):
            eta.csvoutput(etawriter)
        if (time == 4500):
            sys.exit()
    sys.stdout.flush()
    traci.close()

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
    seed(100)
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    net = 'exam1no.net.xml'
    communication_range = 100
    penetration_rate = 0.5
    CVdensi = communication_range*penetration_rate
    comme = "maji"
    f = open("infoResult20190726_CR" + str(communication_range) + "_MPR" + str(penetration_rate) + "_CVden"+ str(CVdensi) + "_eta0.01" + comme + ".csv","w")
    csvWriter = csv.writer(f)
    ff = open("eta20190726_CR" + str(communication_range) + "_MPR" + str(penetration_rate) +  "_CVden"+ str(CVdensi) + "_eta0.01" + comme + ".csv","w")
    etawriter = csv.writer(ff)
    fff = open("Node20190726_CR" + str(communication_range) + "_MPR" + str(penetration_rate) + "_CVden"+ str(CVdensi) + "_eta0.01" + comme + ".csv","w")
    nodewriter = csv.writer(fff)
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, '-c', 'exam1no.sumocfg', '--queue-output', 'queue.xml'])
    run()
    f.close()
