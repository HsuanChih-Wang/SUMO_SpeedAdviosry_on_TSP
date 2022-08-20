import traci
import math
import copy
from scipy.stats import norm
from modules import RSUObject

phaseStrDict = {0: "J1", 1: "J2", 2: "J3", 3: "J4",4: "J5", 5: "J6", 6: "J7", 7: "J8"}
phaseStrDict_rev = {"J1": 0, "J2":1, "J3":2, "J4":3, "J5":4, "J6":5, "J7":6, "J8":7}
phaseLogicPairNum = {0: 0, 1: 0, 2: 3, 3: 3, 4: 6, 5: 6, 6: 9, 7: 9}

intersectionList = ['I1']

# free flow speed
Vf = 12

# Signal Parameters
M = 9999999
GREEN_EXTENT_LIMIT = 10
RED_TRUNCATION_LIMIT = 10
PASS_PROB_THRESHOLD = 0.8

# Bus parameters
BUS_ACTIVATION_SPEED_THRESHOLD = 5


class simTime_less_accCycle_error(Exception):
    def __init__(self,msg):
        self.message=msg

class BusRSU(RSUObject.RSU):
    flag = True
    originalPlan = 0

    def __init__(self, ID, location, detectionRange):
        super().__init__(ID, location, detectionRange)


    def addPlan(self, plan):
        self.plan.append(plan)

    def setOriginalPlan(self):
        self.originalPlan = copy.deepcopy(self.plan)

    def resumePlan(self):
        self.plan = copy.deepcopy(self.originalPlan)

    def getVehicleParameters(self, vehID):
        return super().getVehicleParameters(vehID=vehID)

    def cleanQueueList(self):
        super().cleanQueueList()

    def updateCycleAccumulated(self):
        nowPhase = traci.trafficlight.getPhase('I1')
        nowProgramLogic = traci.trafficlight.getAllProgramLogics('I1')
        if (nowPhase in (11,23) and self.flag == True):
            #The cycle ends
            for phase in nowProgramLogic[0].phases:
                phaseDuration = phase.duration
                self.CycleAccumulated = self.CycleAccumulated + phaseDuration
            self.flag = False

            if (nowPhase == 23 and traci.trafficlight.getProgram('I1') == '1'): # switch TLS program from '1' to '0'
                traci.trafficlight.setProgram('I1', '0')

        else:
            self.flag = True

    # override
    def calPhaseTimeSplit(self, targetPhase):
        '''Argument description
        targetPhase => The phase number that is used for calculation (string)
       '''
        phaseTimeSplit = []

        # Check if the targetphase is the head phase (J1, J5)
        if targetPhase in ['J1', 'J5']:
            IsHeadPhase = True
            # HeadPhase phaseSplit = [(0) Red starts, (1) Red ends, (2)Red starts,..., (n)Red ends]

            #### Calculation cycle K ####
            # Expect: plan[0][targetPhase].startTime = 0
            targetPhaseEndTime = self.plan[0].phases[targetPhase].startTime + self.plan[0].phases[targetPhase].green
            phaseTimeSplit.append(targetPhaseEndTime)  # Add: The ending time of the phase = (0) Red starts.

            #### Calculation cycle K+1 ####
            targetPhaseStartTime = self.plan[1].phases[targetPhase].startTime  # Add: The starting time of the phase = (1) Red ends
            phaseTimeSplit.append(targetPhaseStartTime)
            phaseTimeSplit.append(targetPhaseStartTime + self.plan[1].phases[targetPhase].green)
            #Add: The end of the phase = (2) Red starts

            #### Calculation K+n ####
            cycle = self.plan[2].cycle

            for num in range(2, 5):  # 這裡修改可設定一次產生幾個週期(k+2 k+3...)後的時間
                if (num == 2):
                    phaseTimeSplit.append(self.plan[2].phases[targetPhase].startTime)  # 時相在週期k+2的起始時間 = (1) 紅燈結束
                    startPoint = self.plan[2].phases[targetPhase].startTime
                phaseTimeSplit.append(startPoint + self.plan[2].phases[targetPhase].green)  # 時相在週期k+n的結束時間 = (2) 紅燈起始
                startPoint = startPoint + cycle  # 再加一個週期 -> 時相在k+n的起始時間 = (n) 紅燈結束
                phaseTimeSplit.append(startPoint)


        elif targetPhase in ['J2', 'J3', 'J4', 'J6', 'J7', 'J8']:  # 其他時相編號 (非起頭時相)
            IsHeadPhase = False
            phaseTimeSplit.append(0)  # 在最開始處新增0 (表示由0秒處紅燈起始)
            # 非起頭時相 phaseSplit = [(0) 0, (1) 紅燈結束, (2) 紅燈起始, (3)紅燈結束, (4)紅燈起始, ... , (n)紅燈結束]
            #### 計算週期K ####
            startPoint = self.plan[0].phases[targetPhase].startTime  # targetPhase的起始時間 ( = (1) 紅燈結束時間)
            phaseTimeSplit.append(startPoint)
            phaseTimeSplit.append(startPoint + self.plan[0].phases[targetPhase].green)  # targetPhase結束時間 ( = (2) 紅燈起始時間)

            #### 計算週期K+1 ####
            startPoint = self.plan[1].phases[targetPhase].startTime  # targetPhase 起始時間 = (3) 紅燈結束時間
            phaseTimeSplit.append(startPoint)
            phaseTimeSplit.append(startPoint + self.plan[1].phases[targetPhase].green)  # targetPhase結束時間 ( = (4) 紅燈起始時間)

            #### 計算週期K+n ####
            # 計算k+1週期長度 Expect: cycle = 57
            cycle = self.plan[2].cycle
            # cycle = (plan[1]['J4'].startTime + plan[1]['J4'].green + plan[1]['J4'].yellow + plan[1]['J4'].allRed) - (plan[1]['J1'].startTime)

            for num in range(2, 5):  # 這裡修改可設定一次產生幾個週期( k+2 k+3...)後的時間
                if (num == 2):
                    startPoint = self.plan[2].phases[targetPhase].startTime  # k+1週期的targetPhase起始時間 = (3) 紅燈結束
                    phaseTimeSplit.append(startPoint)
                phaseTimeSplit.append(startPoint + self.plan[2].phases[targetPhase].green)  # k+1週期的targetPhase結束時間 = (4) 紅燈起始
                startPoint = startPoint + cycle  # k+2週期targetPhase的起始時間 = (n) 紅燈結束
                phaseTimeSplit.append(startPoint)
        else:  # 例外錯誤
            print("例外錯誤:　targetPhase = ", targetPhase)

        return phaseTimeSplit, IsHeadPhase  # 回傳 [0] 時間分割點 [1] 是否為起頭時相

    def calPassProb(self, OBU):  # 傳入路口RSU物件
        result = OBU.findTargetIntersections() # 1. 確認公車前方是否還有路口
        if (result == True): # 公車前方還有路口
            # self.location[0] = 取RSU的x座標  / OBU.position[0] = 取車輛的x座標並轉換為整數型態
            if OBU.direction in ['East','West']: # 東西向
                dist = round(abs(self.location[0] - OBU.position[0]))  # 計算到路口的距離
            elif OBU.direction in ['Nort','Sout']: # 南北向
                dist = round(abs(OBU.position[0]))  # 計算到路口的距離

            def resetPassProb(OBU):
                for i in intersectionList:
                    OBU.intersectionPassProb[i] = 1

            # 1. 重設passProb
            # 每次計算前需先重設路口通過機率，否則會重複計算，使通過機率不斷減少
            resetPassProb(OBU)

            try:
                travelTime = round(dist / Vf)  # 至路口旅行時間
            except ZeroDivisionError as zeroDivision:
                print("公車已經停止(除0速度錯誤)，無法計算通過機率")
                # 公車已經停止，不需要計算駕駛建議
                #OBU.recommendSpeed = False
                return False  # 紀錄為False後再return

            # (模擬絕對時間 - 累計週期長度) = 本周期起始時間
            try:
                if (self.CycleAccumulated > traci.simulation.getTime()):
                    raise simTime_less_accCycle_error('注意! 例外錯誤: simTime < CycleAccumulated')
                else:
                    # 抵達路口絕對時間 = 至路口旅行時間 + (模擬絕對時間 - 累計週期長度)
                    arrivalTime = travelTime + traci.simulation.getTime() - self.CycleAccumulated
            except simTime_less_accCycle_error as err:
                print(err.message)
                arrivalTime = travelTime + traci.simulation.getTime() - self.CycleAccumulated

            #print(arrival time = ", arrivalTime)
            deviation = math.ceil((2 + dist / 50) / 3)  # 標準差
            OBU.latestArrivalTime = round(arrivalTime + 3 * deviation)  # 抵達時間上界
            OBU.earliestArrivalTime = round(arrivalTime - 3 * deviation)  # 抵達時間下界
            OBU.arrivalTimeBound = [b for b in range(OBU.earliestArrivalTime, OBU.latestArrivalTime)]  # 列出預計抵達時間離散化範圍

            # 計算未來時制計畫紅燈與綠燈時間點(phaseSplitTime)
            # PhaseObject.calSpecificPhaseTimeSplit 回傳 [0].時間分割點 [1].是否為起頭時相
            phaseTimeSplitResult = self.calPhaseTimeSplit(OBU.targetPhase)
            phaseSplitTime = [round(time) for time in phaseTimeSplitResult[0]]  # 轉換為整數
            currentPhaseGreen = phaseTimeSplitResult[1]
            #print("currentPhaseGreen = ", currentPhaseGreen)
            #print("phaseSplitTime = ", phaseSplitTime)

            ### 指出紅燈區段是哪些時間點 ###
            redBound = []

            # phaseSplit兩兩一組，各自產生離散化範圍
            for n in range(0, len(phaseSplitTime), 2):
                redBound.extend([time for time in range(phaseSplitTime[n], phaseSplitTime[n + 1] + 1)])  # 列出紅燈的時間範圍
            # print("離散化的紅燈秒數區間 = ", redBound)

            ### 計算路口i的通過機率 ###
            # 計算紅燈區段和抵達時間區段之交集
            intersectResult = list(set(OBU.arrivalTimeBound).intersection(set(redBound)))
            intersectResult.sort()  # 因為SET取交集可能使其沒有排序好
            redTimeRange = []  # 紀錄取交集後之紅燈時間長度
            redProbSet = []  # 紀錄紅燈機率

            # 交集後可能包含區間橫跨兩個時相，因此需要個別指出
            for num in range(1, len(intersectResult)):  # 比較在取交集之集合中，後一個數字是否是前一個+1
                if (intersectResult[num] == intersectResult[num - 1] + 1):
                    # 若是，則表示還在原本的切割範圍，繼續加至紅燈時間長度
                    redTimeRange.append(intersectResult[num - 1])
                else:
                    # 若否，表示num已經是新的切割區域
                    redTimeRange.append(intersectResult[num - 1])  # 先將num-1加至舊的切割區域
                    # 計算舊的切割區域之(紅燈)機率
                    redProb = norm.cdf(x=max(redTimeRange), loc=arrivalTime, scale=deviation) \
                              - norm.cdf(x=min(redTimeRange), loc=arrivalTime, scale=deviation)
                    redProbSet.append(redProb)  # 將計算結果apped到redProbSet
                    redTimeRange.clear()  # 清除紅燈計算範圍

                if (num == len(intersectResult) - 1 and len(redTimeRange) > 0):
                    # 最後一部分: 若已經到intersectionResult底 且 redTimeRange還有沒被清空的部分
                    redTimeRange.append(intersectResult[num])
                    redProb = norm.cdf(x=max(redTimeRange), loc=arrivalTime, scale=deviation) \
                              - norm.cdf(x=min(redTimeRange), loc=arrivalTime, scale=deviation)
                    redProbSet.append(redProb)

            # 通過機率 = 1 - (紅燈機率) #
            result = 1
            for prob in redProbSet:
                # 將路口通過機率寫入
                result = result - prob
            return result
        else:
            # result = False 公車前方已經沒有路口
            return False

    def updatePlanParameter(self):

        for num in range(len(self.plan)):
            #expected: num = 0 and 1
            for j in range(0, len(['J1','J2','J3','J4','J5','J6','J7','J8'])):
                if (num == 0):
                    if (j not in [0,4]):
                        self.plan[0].phases[phaseStrDict[j]].startTime = self.plan[0].phases[phaseStrDict[j-1]].startTime + self.plan[0].phases[phaseStrDict[j-1]].green + self.plan[0].phases[phaseStrDict[j-1]].yellow + self.plan[0].phases[phaseStrDict[j-1]].allRed
                elif(num > 0):
                    if (j not in [0,4]):
                        self.plan[num].phases[phaseStrDict[j]].startTime = self.plan[num].phases[phaseStrDict[j - 1]].startTime + self.plan[num].phases[phaseStrDict[j - 1]].green + self.plan[num].phases[phaseStrDict[j-1]].yellow + self.plan[num].phases[phaseStrDict[j-1]].allRed
                    else: # j in [0,4]  / [0] = [3].g + [3].y + [3].AR
                        self.plan[num].phases[phaseStrDict[j]].startTime = self.plan[num-1].phases[phaseStrDict[j+3]].startTime + self.plan[num-1].phases[phaseStrDict[j+3]].green + self.plan[num-1].phases[phaseStrDict[j+3]].yellow + self.plan[num-1].phases[phaseStrDict[j+3]].allRed

                else:
                    print("例外錯誤 num = ", num)


    def adjSignalPlan(self, OBU, green_extent, red_truncate):

        targetPhase_int = phaseStrDict_rev[OBU.targetPhase]  # targetPhase: str -> int


        if OBU.direction in ['East', 'West']:  # 東西向
            dist = round(abs(self.location[0] - OBU.position[0]))  # 計算到路口的距離
        elif OBU.direction in ['Nort', 'Sout']:  # 南北向
            dist = round(abs(OBU.position[0]))  # 計算到路口的距離

        absoluteArrivalTime = traci.simulation.getTime() + (dist / Vf)
        relativeArrivalTime = absoluteArrivalTime - self.CycleAccumulated
        result = self.calPhaseTimeSplit(OBU.targetPhase)
        phaseTimeSplit = result[0]
        IsHeadPhase = result[1]

        deviation = math.ceil((2 + dist / 50) / 3)  # 標準差
        upBound = round(relativeArrivalTime + 3 * deviation)  # 抵達時間上界
        lowBound = round(relativeArrivalTime - 3 * deviation)  # 抵達時間下界
        arrivalBound = [b for b in range(lowBound, upBound + 1)]  # 列出預計抵達時間離散化範圍



        def extentGreen(self, targetPhase, num_Of_plan, greenExtent):
            # expected: targetPhase = 'J3' (string)
            # targetPhase_int = phaseStrDict_rev[targetPhase]
            # currentPhase = traci.trafficlight.getPhase('I1')  # 取得當下執行的時相編號

            # if (targetPhase_int == currentPhase):  # 確認當下運作時相是否就是目標時相
            #     phaseRemainTime = traci.trafficlight.getNextSwitch('I1') - traci.simulation.getTime()
            #     traci.trafficlight.setPhaseDuration('I1', phaseRemainTime + greenExtent)  # 延長當下綠燈秒數
            #
            #     if targetPhase == 'J3':  # 綠燈延長
            #         self.plan[0].phases[targetPhase].green = self.plan[0].phases[OBU.targetPhase].green + greenExtent
            #         self.plan[0].phases['J7'].green = self.plan[0].phases['J7'].green + greenExtent
            #     elif targetPhase == 'J7':
            #         self.plan[0].phases[targetPhase].green = self.plan[0].phases[OBU.targetPhase].green + greenExtent
            #         self.plan[0].phases['J3'].green = self.plan[0].phases['J3'].green + greenExtent
            #     else:
            #         # 尚未完成
            #         print(1 / 0)
            #
            # else:  # 當下運作時相非目標時相 -> 修改Logic物件

            if targetPhase == 'J3':  # 綠燈延長
                self.plan[num_Of_plan].phases[targetPhase].green = self.plan[num_Of_plan].phases[OBU.targetPhase].green + greenExtent
                self.plan[num_Of_plan].phases['J7'].green = self.plan[num_Of_plan].phases['J7'].green + greenExtent
            elif targetPhase == 'J7':
                self.plan[num_Of_plan].phases[targetPhase].green = self.plan[num_Of_plan].phases[OBU.targetPhase].green + greenExtent
                self.plan[num_Of_plan].phases['J3'].green = self.plan[num_Of_plan].phases['J3'].green + greenExtent
            else:
                # 尚未完成
                print(1 / 0)
            pass

        def truncateRed(self, num_Of_Plan, num_Of_Phase, redTruncate):
            self.plan[num_Of_Plan].phases[phaseStrDict[num_Of_Phase]].green = self.plan[num_Of_Plan].phases[phaseStrDict[num_Of_Phase]].green - redTruncate



        if (IsHeadPhase == False):
            # 非起頭時相 phaseSplit = [(0) 0, (1) 紅燈結束, (2) 紅燈起始, (3)紅燈結束, (4)紅燈起始, ... , (n)紅燈結束]

            for num in phaseTimeSplit:  # 找出抵達時落在第幾個週期
                if upBound <= num:
                    upBoundIndex2 = phaseTimeSplit.index(num)
                    upBoundIndex1 = upBoundIndex2 - 1
                    break

            for num in phaseTimeSplit:  # 找出抵達時落在第幾個週期
                if lowBound <= num:
                    lowBoundIndex2 = phaseTimeSplit.index(num)
                    lowBoundIndex1 = lowBoundIndex2 - 1
                    break

            if (lowBoundIndex2 == 1 and lowBoundIndex1 == 0 and upBoundIndex2 == 1 and upBoundIndex1 == 0):
                #case 1: cycle[0] 單純 紅燈縮短
                #targetPhase_int - 2 = phase 0 # targetPhase_int - 2 + 4 = phase 4
                case = 1
                if (targetPhase_int in [2,6]):
                    truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=0, redTruncate=red_truncate)
                    truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=4, redTruncate=red_truncate)
                else:
                    # 其他方向尚未設定
                    pass

            elif (lowBoundIndex1 == 0 and lowBoundIndex2 == 1 and upBoundIndex1 == 1 and upBoundIndex2 == 2):
                # case 2: cycle[0] 紅燈縮短 或 綠燈延長
                case = 2
                if (targetPhase_int in [2, 6]): # 紅燈縮短
                    truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=0, redTruncate=red_truncate)
                    truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=4, redTruncate=red_truncate)
                else:
                    # 其他方向尚未設定
                    pass

                # 綠燈延長
                extentGreen(self=self, num_Of_plan=0, targetPhase=OBU.targetPhase, greenExtent=green_extent)

            elif (lowBoundIndex1 == 1 and lowBoundIndex2 == 2 and upBoundIndex1 == 2 and upBoundIndex2 == 3):
                # case 3: cycle[0] 單純 綠燈延長
                case = 3
                # 綠燈延長
                extentGreen(self=self, num_Of_plan=0, targetPhase=OBU.targetPhase, greenExtent=green_extent)

            elif (lowBoundIndex1 == 2 and lowBoundIndex2 == 3 and upBoundIndex1 == 2 and upBoundIndex2 == 3):
                if (relativeArrivalTime < self.plan[0].cycle):
                    # case 4: cycle [0] 單純綠燈延長
                    case = 4
                    extentGreen(self=self, num_Of_plan=0, targetPhase=OBU.targetPhase, greenExtent=green_extent)

                else:
                    # case 5: cycle [1] 紅燈縮短
                    case = 5
                    if (targetPhase_int in [2, 6]):
                        # cycle 1 紅燈縮短
                        truncateRed(self=self, num_Of_Plan=1, num_Of_Phase=0, redTruncate=red_truncate)
                        truncateRed(self=self, num_Of_Plan=1, num_Of_Phase=4, redTruncate=red_truncate)
                        # # cycle 0 紅燈縮短
                        truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=0, redTruncate=red_truncate)
                        truncateRed(self=self, num_Of_Plan=0, num_Of_Phase=4, redTruncate=red_truncate)
                    else:
                        # 其他方向尚未設定
                        print(1/0)
                        pass

            elif (lowBoundIndex1 == 2 and lowBoundIndex2 == 3 and upBoundIndex1 == 3 and upBoundIndex2 == 4):
                # case 6 : cycle[1] 紅燈縮短 或 綠燈延長
                case = 6

                if (targetPhase_int in [2, 6]):  # 紅燈縮短
                    truncateRed(self=self, num_Of_Plan=1, num_Of_Phase=0, redTruncate=red_truncate)
                    truncateRed(self=self, num_Of_Plan=1, num_Of_Phase=4, redTruncate=red_truncate)
                else:
                    # 其他方向尚未設定
                    pass

                # 綠燈延長
                extentGreen(self=self, num_Of_plan=1, targetPhase=OBU.targetPhase, greenExtent=green_extent)

            elif (lowBoundIndex1 == 3 and lowBoundIndex2 == 4 and upBoundIndex1 == 4 and upBoundIndex2 == 5):
                #case 7: cycle[1] 單純 綠燈延長
                case = 7
                # 綠燈延長
                extentGreen(self=self, num_Of_plan=1, targetPhase=OBU.targetPhase, greenExtent=green_extent)

            elif (lowBoundIndex1 == 4 and lowBoundIndex2 == 5 and upBoundIndex1 == 4 and upBoundIndex2 == 5):
                # case 8: cycle[1] 單純 綠燈延長
                case = 8
                # 綠燈延長
                extentGreen(self=self, num_Of_plan=1, targetPhase=OBU.targetPhase, greenExtent=green_extent)

            elif (lowBoundIndex1 == 3 and lowBoundIndex2 == 4 and upBoundIndex1 == 3 and upBoundIndex2 == 4):
                #抵達範圍全部是綠燈
                #print("抵達範圍全部是綠燈，不用綠燈延長或紅燈縮短")
                print(1 / 0)  # 強制產生除0錯誤 終止程式

            elif (lowBoundIndex1 == 1 and lowBoundIndex2 == 2 and upBoundIndex1 == 1 and upBoundIndex2 == 2):
                #抵達範圍全部是綠燈 有bug
                #print("抵達範圍全部是綠燈，不用綠燈延長或紅燈縮短")
                print(1/0) #強制產生除0錯誤 終止程式

            else:
                print("不屬於任何一類! 錯誤!")
                print("lowBoundIndex1 = %d / lowBoundIndex2 = %d / upBoundIndex1 = %d / upBoundIndex2 = %d" % (lowBoundIndex1, lowBoundIndex2, upBoundIndex1, upBoundIndex2))
                print("upBound = %d / lowBound = %d" %(upBound, lowBound))
                print("phaseTimeSplit = ", phaseTimeSplit)
                print(1/0) #強制跳出
                # BUG IS HERE !
                #*return False

        else:
            print("IsHeadPhase == ", IsHeadPhase)
            print("暫時沒有處理")

        self.updatePlanParameter()  # 更新plan參數
        return case
        # num_of_cycle = relativeArrivalTime % self.plan[0].cycle
        # Expected: num_of_cycle is either 0 or 1

        # phase = phaseLogicPairNum[phase] # 找logic物件中對應phase
        # nowProgramLogic = traci.trafficlight.getAllProgramLogics('I1') # 取得目前program logic物件
        # nowProgramLogic[num_of_cycle].phases[phase].duration = nowProgramLogic[num_of_cycle].phases[phase].duration + green_extent # 綠燈延長
        # nowProgramLogic[num_of_cycle].phases[phase-2].duration = nowProgramLogic[num_of_cycle].phases[phase-2].duration - red_truncate # 紅燈縮短


    def calTSPstrategy(self, OBU):
        totalAdjustment = M
        tempProbability = 0
        totalGreenExtention = 0
        totalRedTruncation = 0
        # plan_backup = []

        # # 備份原本plan內容 (時制編號2)
        # plan_backup.append(copy.deepcopy(self.plan[2]))
        # plan_backup.append(copy.deepcopy(self.plan[2]))

        currentProb = self.calPassProb(OBU=OBU)

        if (OBU.currentSpeed > BUS_ACTIVATION_SPEED_THRESHOLD):  # 公車時速過低時不執行優先策略
            if currentProb < PASS_PROB_THRESHOLD:
                for i in range(1, GREEN_EXTENT_LIMIT + 1):
                    for j in range(1, RED_TRUNCATION_LIMIT + 1):
                        # 修改plan
                        self.adjSignalPlan(OBU, i, j)
                        tempProbability = self.calPassProb(OBU=OBU)
                        if tempProbability > PASS_PROB_THRESHOLD:
                            if (i + j) < totalAdjustment:
                                totalAdjustment = i + j
                                totalGreenExtention = i
                                totalRedTruncation = j
                        else:
                            print("通過機率未達標準!")
                            print("綠燈延長 %d / 紅燈切斷 %d / tempProbability = %f / PASS_PROB_THRESHOLD = %f " % (i, j, tempProbability, PASS_PROB_THRESHOLD))
                        # 復原原時制計畫
                        self.resumePlan()

                if totalAdjustment < M:  # 設定新時制策略
                    print("優先策略：總調整秒數 %d 秒 / 綠燈延長 %d 秒 / 紅燈縮短 %d 秒" % (
                    totalAdjustment, totalGreenExtention, totalRedTruncation))

                    self.adjSignalPlan(OBU=OBU, green_extent=totalGreenExtention,
                                              red_truncate=totalRedTruncation)

                else:  # 沒有找到可行策略
                    # 採Plan B: 公車延滯最小 (紅燈縮短)
                    print("沒有找到可行策略，採用Plan B: 公車延滯最小 (紅燈縮短)")
                    case = self.adjSignalPlan(OBU=OBU, green_extent=0, red_truncate=0)  # 目的: 取得case

                    if case in [1, 2]:
                        self.plan[0].phases[phaseStrDict[0]].green = self.plan[1].phases[phaseStrDict[0]].green - RED_TRUNCATION_LIMIT
                        self.plan[0].phases[phaseStrDict[4]].green = self.plan[1].phases[phaseStrDict[4]].green - RED_TRUNCATION_LIMIT
                    elif case in [3, 4, 5, 6, 7, 8]:
                        self.plan[1].phases[phaseStrDict[0]].green = self.plan[1].phases[phaseStrDict[0]].green - RED_TRUNCATION_LIMIT
                        self.plan[1].phases[phaseStrDict[4]].green = self.plan[1].phases[phaseStrDict[4]].green - RED_TRUNCATION_LIMIT

                    else:
                        print("case = ", case)
                        print(1 / 0)  # 例外錯誤

                # BUG!!! in old make logic object
                print("self.plan[0] = ", self.plan[0])
                print("self.plan[1] = ", self.plan[1])
                newProgram0 = self.plan[0].OLDmakeLogicObject()  # 產生logic物件
                newProgram1 = self.plan[1].OLDmakeLogicObject()  # bug is in plan[1]
                print("newProgram0 = ", newProgram0)
                for phase in newProgram1.phases:
                    newProgram0.phases.append(phase)
                print("newProgram0 = ", newProgram0)

                nowPhase = traci.trafficlight.getPhase('I1')

                traci.trafficlight.setProgramLogic('I1', newProgram0)
                traci.trafficlight.setProgram('I1', '1')  # 切換到新設定program
                traci.trafficlight.setPhase('I1', nowPhase)  # 設定仍維持執行當下時相

            else:
                print("目前時制配置通過機率足夠! 不計算優先策略")

            OBU.set_obu_tsp_indicator(0)
            print("OBD ID = %s / OBU obu_tsp_indicator = %d" %(OBU.OBU_ID, OBU.OBU_TSP_INDICATOR))
            print("")



    def __str__(self):
        return 'Bus RSU(ID = {0}, location = {1},  plan = {2})'\
            .format(self.RSU_ID, self.location, self.plan)





