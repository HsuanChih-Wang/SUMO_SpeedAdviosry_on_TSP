import traci
import math
from scipy.stats import norm

# !!!! 多路口記得檢查修改 !!!!!
intersectionList = ['I1']

OBU_START_INDICATOR = 1

BETA = 0.05

class simTime_less_accCycle_error(Exception):
    def __init__(self,msg):
        self.message=msg

class OBU:
    ### Settings ###
    passProbThreshold = 0.5  # 路口通過機率門檻
    maxSpeedLimit = 13.89 # 約 50 km/hr
    MaxSpeedFactor = 0.1
    MinSpeedFactor = 0.3

    def __init__(self,ID,vehType,pos,currentSpeed,direction,nextTLS,targetPhase):
        self.OBU_ID = ID
        self.vehType = vehType
        self.position = pos
        self.currentSpeed = currentSpeed
        self.direction = direction
        self.nextTLS = nextTLS
        self.targetPhase = targetPhase

        self.targetIntersectionList = []  # 目標路口 (Host RSU)
        self.needOptIntersectionList = []  # 需要計算駕駛建議的路口
        self.intersectionPassProb = {}  # 紀錄路口i的通過機率，初始值為100%
        self.recommendSpeed = 0

    def start(self, RSUs): #傳入各路口RSU物件(字典型態)
        if (OBU_START_INDICATOR == 1): #控制是否要執行駕駛建議
            # RSUs = {'I1':I1(RSU物件), 'I2':I2(RSU物件), ... }
            # 0. 初始化: 將相關屬性重設
            self.needOptIntersectionList = []
            self.targetIntersectionList = []
            # 1. 找targetIntersections
            result = self.findTargetIntersections()
            if (result): #若回傳為true才繼續，否則不執行以下
                # 2. 計算各路口通過機率
                for i in self.targetIntersectionList:
                    passProb = self.calPassProb(RSUs[i])
                    if (passProb != False): #False條件: 公車停止
                        self.intersectionPassProb.update({i:passProb}) # 更新intersectionPassProb list
                        self.needOptIntersectionList.append(i) # 將路口加入需要計算路口列表中

                        if (passProb <= self.passProbThreshold):  # 若通過機率小於門檻值
                            print("路口[ %s ]通過機率 = %f < = 目標機率 %f，需要計算建議速度" % (i, self.intersectionPassProb[i], self.passProbThreshold))
                            self.__calOptimalSpeed(RSUs, passProb)  # 計算駕駛建議
                            break
                        else: # 若通過機率大於門檻值，不用計算建議速度
                            print("路口[ %s ]通過機率 = %f > 目標機率 %f，不用計算建議速度" % (i, self.intersectionPassProb[i], self.passProbThreshold))
                            traci.vehicle.setSpeed(self.OBU_ID, -1)  # 只要滿足目標機率 就把速度控制權還給SUMO
                            # 檢查i是否已經是self.targetIntersectionList最後一個
                            if ( self.targetIntersectionList.index(i) == (len(self.targetIntersectionList) - 1)):
                                return True
                            else:
                                pass
                    else:
                        # expected: passProb = False
                        # 公車已停止，不需要速度建議
                        return False

                # 檢查是否有計算出新的建議速度
                # if (self.recommendSpeed != 0):
                #     # recommendSpeed != 0 表示窮舉中有速度之通過路口機率有改善
                #     # Case 1: 採建議速度
                #     optimalSpeed = self.recommendSpeed
                #     print("OBU ID = %s +++++++++ Optimal speed = %d m/s ++++++++" % (self.OBU_ID, optimalSpeed))
                #     traci.vehicle.setSpeed(self.OBU_ID, optimalSpeed)
                # else:
                #     # Case 2: 採原速度前進
                #     print("所有建議速度均沒有比原速好，照原本速度前進 (速度控制權還給SUMO)")
                #     traci.vehicle.setSpeed(self.OBU_ID, -1)

                optimalSpeed = self.recommendSpeed
                if optimalSpeed < 10:
                    print("因為 optimalSpeed < 10 已更改為 10")
                    optimalSpeed = 10
                print("OBU ID = %s +++++++++ Optimal speed = %d m/s ++++++++" % (self.OBU_ID, optimalSpeed))
                traci.vehicle.setSpeed(self.OBU_ID, optimalSpeed)

            else:
                print("self.findTargetIntersections() return %s" % result)
                print("OBU: %s 公車即將離開路網，不用計算建議速度，將速度控制權還給SUMO" % self.OBU_ID)
                # 因不確定是否車輛還有受速度控制，統一下指令將控制權還給SUMO
                traci.vehicle.setSpeed(self.OBU_ID, -1)  # 引數-1 表示將控制權還給SUMO
        else:
            print("OBU_START_INDICATOR != 1 沒有開啟駕駛建議功能")
            return True

    def findTargetIntersections(self):
        #  1. 先確認公車是否仍有下一個路口
        if (self.nextTLS != None):
            # 若有，紀錄為計算路口群組之起始路口(startIntersectionIndex)
            startIntersectionIndex = intersectionList.index(self.nextTLS)  # 取出該路口編號之index標籤
        else:  # 若無，則代表公車即將離開路網，不需要計算駕駛建議
            #self.recommendSpeed = False  # 紀錄為False後再return
            return False

        # 2. 識別公車方向
        if self.direction == 'East':
            print("公車方向: 向東")
            # 3. 找出targetIntersection
            for i in range(startIntersectionIndex, len(intersectionList)):  # 從群組之起始路口開始往後算，列舉出每個路口即為target Intersection
                i_Str = intersectionList[i]  # 取出路口str型態名稱 ex. I1 I2
                self.targetIntersectionList.append(i_Str)  # 將路口列入targetIntersectionList
                self.intersectionPassProb.update({i_Str: 1})  # 更新路口加入通過機率計算(預設機率=1)

            return True

        elif self.direction == 'West':
            print("公車方向: 向西")
            for i in range(startIntersectionIndex, -1, -1):
                i_Str = intersectionList[i]  # 取出路口str型態名稱 ex. I1 I2
                self.targetIntersectionList.append(i_Str)  # 將路口列入targetIntersectionList
                self.intersectionPassProb.update({i_Str: 1})  # 將路口加入通過機率計算(預設機率=1)

            return True

        elif (self.direction == 'North' or self.direction == 'South'):  # 因為veh type擷取只有前4位字元
            print("公車方向: 向北 / 向南")
            i_Str = self.nextTLS
            self.targetIntersectionList.append(i_Str)
            self.intersectionPassProb.update({i_Str: 1})  # 將路口加入通過機率計算(預設機率=1)
            # # self.position[1] = -200 -> -188 -> -165 ...
            # dist = round(abs(self.position[1]))  # self.position[1] 取y座標並轉換為整數型態
            # passProb_of_i(self, RSUs[i_Str], dist)
            return True
        else:
            print("例外錯誤! 公車方向不屬於任何一方")
            print(1/0) #程式強制中止
            return False

    def calPassProb(self, RSU):  # 傳入路口RSU物件
        # ex. RSUs = {'I1':I1(RSU物件), 'I2':I2(RSU物件) ... }
        # RSUs[i].location[0] = 取RSU的x座標  / self.position[0] = 取車輛的x座標並轉換為整數型態 / self.position[1] = 取車輛的y座標
        if self.direction in ['East','West']: # 東西向
            dist = round(abs(RSU.location[0] - self.position[0]))  # 計算到路口的距離
        elif self.direction in ['North','South']: # 南北向
            dist = round(abs(self.position[1]))  # 計算到路口的距離

        # 1. 重設passProb
        # 每次計算前需先重設路口通過機率為1，否則會重複計算，使通過機率不斷減少
        self.intersectionPassProb[RSU.RSU_ID] = 1

        try:
            travelTime = round(dist / self.currentSpeed)  # 至路口旅行時間
        except ZeroDivisionError as zeroDivision:
            print("公車已經停止(除0速度錯誤)，不需要計算駕駛建議")
            # 公車已經停止，不需要計算駕駛建議
            # self.recommendSpeed = False
            return False  # 紀錄為False後再return

        # (模擬絕對時間 - 累計週期長度) = 本周期起始時間
        try:
            if (RSU.CycleAccumulated > traci.simulation.getTime()):
                raise simTime_less_accCycle_error('注意! 例外錯誤: simTime < CycleAccumulated')
            else:
                # 抵達路口絕對時間 = 至路口旅行時間 + (模擬絕對時間 - 累計週期長度)
                arrivalTime = travelTime + traci.simulation.getTime() - RSU.CycleAccumulated
        except simTime_less_accCycle_error as err:
            print(err.message)
            arrivalTime = travelTime + traci.simulation.getTime() - RSU.CycleAccumulated

        #print("arrival time = ", arrivalTime)
        deviation = math.ceil((2 + dist / 50) / 3)  # 標準差
        upBound = round(arrivalTime + 3 * deviation)  # 抵達時間上界
        lowBound = round(arrivalTime - 3 * deviation)  # 抵達時間下界
        arrivalBound = [b for b in range(lowBound, upBound + 1)]  # 列出預計抵達時間離散化範圍

        # 計算未來時制計畫紅燈與綠燈時間點(phaseSplitTime)
        # PhaseObject.calSpecificPhaseTimeSplit 回傳 [0].時間分割點 [1].是否為起頭時相
        phaseTimeSplitResult = RSU.calPhaseTimeSplit(self.targetPhase)
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
        intersectResult = list(set(arrivalBound).intersection(set(redBound)))
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

    def __calOptimalSpeed(self, RSUs, PreStepPassProb): #輸入路口RSU物件
        # RSUs = {'I1':I1(RSU物件), 'I2':I2(RSU物件), ... }

        maxPassProb = 0 # 初始化maxPassProb為0       print("self.needOptIntersectionList = ", self.needOptIntersectionList)
        recommendedSpeed = 0 # 建議速度
        # for i in self.needOptIntersectionList: # 累計各路口通過機率
        #     maxPassProb = maxPassProb + self.intersectionPassProb[i]  # 最大通過機率預設為以原始速度通過的機率

        # 計算各速度下的通過機率
        maxSpeed = round((1 + self.MaxSpeedFactor) * self.maxSpeedLimit)  # 設定建議速度上限
        minSpeed = round((1 - self.MinSpeedFactor) * self.maxSpeedLimit)  # 設定建議速度下限

        PreStepSpeed = self.currentSpeed #紀錄當下速度(vbpr)

        for v in range(minSpeed, maxSpeed+1):  # 從 minSpeed 到 maxSpeed 窮舉
            self.currentSpeed = v  # 將目前速度設為v

            totalPassProb = 0 # 初始化路口通過總機率為0
            for i in self.needOptIntersectionList:
                newPassProb = self.calPassProb(RSUs[i])  # 以目前速度計算路口通過機率
                self.intersectionPassProb.update({i: newPassProb})  # 更新路口i的通過機率 intersectionPassProb list
                totalPassProb = totalPassProb + self.intersectionPassProb[i] # 加總各路口通過機率成為totalPassProb

            if (totalPassProb > maxPassProb):  # 取代條件: 新計算結果大於舊的 ，則取代之
                print("速度 %d 之 路口通過總機率為 %f 比原本 %f 好" % (v, totalPassProb, maxPassProb))
                maxPassProb = totalPassProb
                recommendedSpeed = v  # 新速度v作為建議速度
            else:
                print("速度 %d 之 路口通過機率為 %f 沒有比原本 %f 好" % (v, totalPassProb, maxPassProb))

        if ((maxPassProb - PreStepPassProb) > BETA): # 檢查本次建議速度之通過機率: 與前次相比是否大於門檻值(Beta)
            self.recommendSpeed = recommendedSpeed
            print("maxPassProb = ", maxPassProb, "  PreStepPassProb = ", PreStepPassProb)
            print("Case 1   OBU: %s 建議速度 = %d" % (self.OBU_ID, self.recommendSpeed))
        else:
            self.recommendSpeed = PreStepSpeed # 若沒有，則Case 2: 採原速度前進，維持建議前一次速度
            print("Case 2   OBU: %s 建議速度 = %d" % (self.OBU_ID, self.recommendSpeed))

    def __str__(self):
        return 'OBU(OBU_ID = {0}, vehType = {1}, position  = {2}, currentSpeed = {3}, direction = {4}, nextTLS = {5}, targetPhase = {6}, recommendSpeed = {7})'\
            .format(self.OBU_ID, self.vehType, self.position , self.currentSpeed, self.direction, self.nextTLS, self.targetPhase, self.recommendSpeed)



