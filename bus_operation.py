
import os
import sys
import optparse

from modules import SignalPlanObject
from modules import BusRSU
from modules import BusOBU

from sumolib import checkBinary  # Checks for the binary in environ vars
import traci

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

OBU_Dict = {}
RSUs = dict()
signalOPTs = dict()

planHorizon = [0, 1]

def setPhaseObject(i, inputPlan):

    signalPlan = SignalPlanObject.SignalPlan()

    J1 = SignalPlanObject.Phase(phaseID='J1', phaseOrder=0, startTime=inputPlan[1][0], green=inputPlan[0][0], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J2 = SignalPlanObject.Phase(phaseID='J2', phaseOrder=1, startTime=inputPlan[1][1], green=inputPlan[0][1], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J3 = SignalPlanObject.Phase(phaseID='J3', phaseOrder=2, startTime=inputPlan[1][2], green=inputPlan[0][2], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J4 = SignalPlanObject.Phase(phaseID='J4', phaseOrder=3, startTime=inputPlan[1][3], green=inputPlan[0][3], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J5 = SignalPlanObject.Phase(phaseID='J5', phaseOrder=4, startTime=inputPlan[1][4], green=inputPlan[0][4], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J6 = SignalPlanObject.Phase(phaseID='J6', phaseOrder=5, startTime=inputPlan[1][5], green=inputPlan[0][5], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J7 = SignalPlanObject.Phase(phaseID='J7', phaseOrder=6, startTime=inputPlan[1][6], green=inputPlan[0][6], yellow=3, allRed=2, Gmax=200, Gmin=0)
    J8 = SignalPlanObject.Phase(phaseID='J8', phaseOrder=7, startTime=inputPlan[1][7], green=inputPlan[0][7], yellow=3, allRed=2, Gmax=200, Gmin=0)
    R = J1.yellow + J1.allRed
    CYCLE = J1.green + J2.green + J3.green + J4.green + R * 4

    signalPlan.setAllParameters(planID='0', order='00', offset=0, cycle=CYCLE, phases={"J1": J1, "J2": J2, "J3": J3, "J4": J4,
                                                                                         "J5": J5, "J6": J6, "J7": J7, "J8": J8})
    print("signalPlan = ", signalPlan)

    i.addPlan(signalPlan)


def initialization():
    # 初始化: 新增RSU
    I1 = BusRSU.BusRSU(ID='I1', location=[200, 0], detectionRange=200)
    RSUs.update({'I1': I1})

    print(RSUs['I1'].RSU_ID)
    print(RSUs['I1'].location)

    Plan1 = [[34, 11, 43, 12, 34, 11, 43, 12], [0, 39, 55, 103, 0, 39, 55, 103]]
    Plan2 = [[34, 11, 43, 12, 34, 11, 43, 12], [120, 159, 175, 223, 120, 159, 175, 223]]
    originalPlan = [[34, 11, 43, 7, 34, 11, 43, 12], [0, 39, 55, 103, 0, 39, 55, 103]]

    for rsu in RSUs:
        setPhaseObject(RSUs[rsu], Plan1)
        setPhaseObject(RSUs[rsu], Plan2)
        setPhaseObject(RSUs[rsu], originalPlan)
        RSUs[rsu].setOriginalPlan()

# contains TraCI control loop
def run():
    step = 0

    # 號誌設定初始化 SignalPlan Initialization
    initialization()

    while traci.simulation.getMinExpectedNumber() > 0:
        # 迴圈直到所有車輛都已離開路網
        traci.simulationStep()

        print("########################### step = ", step, " ###########################")

        if (step > 3 and step == RSUs['I1'].CycleAccumulated + 2):
            traci.trafficlight.setProgram('I1', '0')  # 切回原本的時制計畫

        # 清除OBU LIST
        #OBU_Dict.clear()

        for rsu in RSUs:  # RSUs = ['rsu':'RSU object']
            RSUs[rsu].cleanQueueList()  # 清除veh queue list
            RSUs[rsu].updateCycleAccumulated()

            # Vehicle
            vehIDlist = traci.vehicle.getIDList()  # 取出路網中所有車輛ID
            for veh in vehIDlist:  # 個別抽出車輛ID
                # 取得該車輛ID的詳細參數  getVehicleParameters
                # Input: VehicleID (String) / Output:  {"vehID": vehID, "order": order, "type": vehType ... }

                vehX = RSUs[rsu].getVehicleParameters(veh)

                # print(vehX)
                if (vehX['type'] == 'ElectricBus'):

                    # 找出車輛型態為公車者，加入OBU字典中，型態：
                    # 車輛ID：駕駛建議(RecommendSpeed)物件

                    if ( (vehX['vehID'] not in OBU_Dict)):
                        OBU_Dict.update({vehX['vehID']: BusOBU.BusOBU(ID=vehX['vehID'], vehType=vehX['type'], pos=vehX['position'], currentSpeed=vehX['vehSpeed'], direction=vehX['direction'],
                                             nextTLS=vehX['nextTLSID'], targetPhase=vehX['phase'])})

                    else:
                        print("同一組OBU: %s  已經在OBU list: %s 中，更新相關參數" % (vehX['vehID'], OBU_Dict))
                        OBU_Dict[vehX['vehID']].setParameters(ID=vehX['vehID'], vehType=vehX['type'], pos=vehX['position'], currentSpeed=vehX['vehSpeed'], direction=vehX['direction'],
                                             nextTLS=vehX['nextTLSID'], targetPhase=vehX['phase'])
                        traci.vehicle.changeLane(vehID=vehX['vehID'], laneIndex=0, duration=999)


                    obu_del_list = []
                    for obu in OBU_Dict:
                        if (OBU_Dict[obu].nextTLS == None):
                            print("OBU: %s 公車即將離開路網，將速度控制權還給SUMO" % OBU_Dict[obu].OBU_ID)
                            # 因不確定是否車輛還有受速度控制，統一下指令將控制權還給SUMO
                            traci.vehicle.setSpeed(OBU_Dict[obu].OBU_ID, -1)  # 引數-1 表示將控制權還給SUMO
                            # OBU_Dict[obu].OBU_RECOMMENDED_SPEED_INDICATOR = 1  # 重新開啟速度建議功能
                            # OBU_Dict[obu].PLAN_C_INDICATOR = False  # 將plan C indicator 關閉

                            obu_del_list.append(obu)

                    for obu in obu_del_list:
                        del OBU_Dict[obu]  # 若 obu 前方已經沒有路口，則刪除該OBU

                    print("OBU_Dict = ", OBU_Dict)


        # 公車優先策略計算
        for rsu in RSUs:
            for obu in OBU_Dict:
                print("before: OBU_Dict[%s].OBU_TSP_INDICATOR = %d" % (obu, OBU_Dict[obu].OBU_TSP_INDICATOR))
                if OBU_Dict[obu].OBU_TSP_INDICATOR == 1:
                    RSUs[rsu].calTSPstrategy(OBU=OBU_Dict[obu])
                    print("after: OBU_Dict[%s].OBU_TSP_INDICATOR = %d" % (obu, OBU_Dict[obu].OBU_TSP_INDICATOR))
                else:
                    print("")

        # 駕駛建議
        if (len(OBU_Dict) > 0):
            print("OBU_DICT = ", OBU_Dict)
            for obu in OBU_Dict:
                OBU_Dict[obu].start(RSUs)
                print("targetPhase =  ", OBU_Dict[obu].targetPhase)

        print("NowProgram = ", traci.trafficlight.getProgram('I1'),
              "/ AllProgramLogic = ", traci.trafficlight.getAllProgramLogics('I1'),
              "/ NowPhaseName = ", traci.trafficlight.getPhaseName('I1'),
              "/ NowPhase = ", traci.trafficlight.getPhase('I1'),
              "/ NowPhaseDuration = ", traci.trafficlight.getPhaseDuration('I1'),
              "/ NowState = ", traci.trafficlight.getRedYellowGreenState('I1'),
              "/ NextSwitch = ", traci.trafficlight.getNextSwitch('I1'), )

        print("cycleAcculmulated = ", RSUs[rsu].CycleAccumulated)


        nowProgram = traci.trafficlight.getAllProgramLogics('I1')
        print("nowProgram = ", nowProgram[0].phases[0])


        step += 1

    traci.close()
    sys.stdout.flush()

# main entry point
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # traci starts sumo as a subprocess and then this script connects and runs
    sumo_start = [sumoBinary, "-c", "bus_operation.sumocfg",
                  "--tripinfo-output", "result/bus_operation_tripInfo.xml",
                  "--emission-output", "result/bus_operation_emissionInfo.xml",
                  "--battery-output", "result/bus_operation_batteryOut.xml",
                  "--summary-output", "result/bus_operation_summary.xml",
                  "--random", "--seed", "8"]  # "--random", "--seed", "8"
                # "--device.emissions.probability 1.0"
    traci.start(sumo_start)
    run()
