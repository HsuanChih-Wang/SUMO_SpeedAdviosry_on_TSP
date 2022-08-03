import traci

from modules import OBUObject


intersectionList = ['I1']

OBU_START_INDICATOR = 1 #公車車機是否啟動


phaseStrDict_rev = {"J1": 0, "J2":1, "J3":2, "J4":3, "J5":4, "J6":5, "J7":6, "J8":7}


class simTime_less_accCycle_error(Exception):
    def __init__(self,msg):
        self.message=msg

class t_less_0_error(Exception):
    def __init__(self,msg):
        self.message = msg

class BusOBU(OBUObject.OBU):

    latestArrivalTime = 0
    earliestArrivalTime = 0
    arrivalTimeBound = 0
    acceleration = 0

    ### Settings ###
    passProbThreshold = 0.8  # 路口通過機率門檻
    maxSpeedLimit = 15  # 約 54 km/hr
    MaxSpeedFactor = 0.1
    MinSpeedFactor = 0.3

    OBU_RECOMMENDED_SPEED_INDICATOR = 0  # 駕駛建議每路口只做一次，做完後=0
    OBU_TSP_INDICATOR = 0 # 公車優先每個路口只做一次，做完後=0
    PLAN_C_INDICATOR = False

    def __init__(self,ID,vehType,pos,currentSpeed,direction,nextTLS,targetPhase):
        super().__init__(ID,vehType,pos,currentSpeed,direction,nextTLS,targetPhase)

    def setParameters(self, ID,vehType,pos,currentSpeed,direction,nextTLS,targetPhase):
        self.OBU_ID = ID
        self.vehType = vehType
        self.position = pos
        self.currentSpeed = currentSpeed
        self.direction = direction
        self.nextTLS = nextTLS
        self.targetPhase = targetPhase

    def set_obu_tsp_indicator(self,indicator):
        self.OBU_TSP_INDICATOR = indicator

    def start(self, RSUs): #傳入各路口RSU物件(字典型態)
        global OBU_START_INDICATOR
        # global OBU_RECOMMENDED_SPEED_INDICATOR
        # global PLAN_C_INDICATOR
        print("OBU_START_INDICATOR = ", OBU_START_INDICATOR)
        print("OBU_RECOMMENDED_SPEED_INDICATOR = ", self.OBU_RECOMMENDED_SPEED_INDICATOR)
        print("PLAN_C_INDICATOR = ", self.PLAN_C_INDICATOR)
        if (OBU_START_INDICATOR == 1): # 公車車機是否啟動
            # RSUs = {'I1':I1(RSU物件), 'I2':I2(RSU物件), ... }
            # 0. 初始化: 將相關屬性重設
            self.needOptIntersectionList = []
            self.targetIntersectionList = []
            # 1. 找targetIntersections
            result = self.findTargetIntersections() # 沿用父類別方法
            if (result): #若回傳為true才繼續，否則不執行以下
                # 2. 計算各路口通過機率
                if (self.OBU_RECOMMENDED_SPEED_INDICATOR == 1 and self.currentSpeed > 10):
                    for i in self.targetIntersectionList:
                        passProbResult = self.calPassProb(RSUs[i]) # 呼叫計算OBU通過機率
                        if (passProbResult != False):  # False = 公車已經停止(除0速度錯誤)，不需要計算駕駛建議
                            self.intersectionPassProb.update({i:passProbResult}) # 更新intersectionPassProb list
                            self.needOptIntersectionList.append(i) # 將路口加入需要計算路口列表中

                            if (passProbResult <= self.passProbThreshold):  # 若通過機率小於門檻值
                                print("路口[ %s ]通過機率 = %f < = 目標機率 %f，需要計算駕駛建議速度" % (i, self.intersectionPassProb[i], self.passProbThreshold))
                                euslt = self.calOptimalSpeed(RSUs)  # 計算駕駛建議
                                break
                            else: # 若通過機率大於門檻值，不用計算建議速度
                                # Plan A
                                print("Plan A: 路口[ %s ]通過機率 = %f > 目標機率 %f，建議速度 = 目前速度" % (i, self.intersectionPassProb[i], self.passProbThreshold))
                                traci.vehicle.setSpeed(self.OBU_ID, -1)
                                return True
                        else:  # if passProbResult == False
                            # 公車已停止
                            # print("passProb = ", passProb)
                            return False

                    # 檢查是否有計算出新的建議速度
                    if (self.recommendSpeed != 0):
                        # Plan B
                        # recommendSpeed != 0 表示窮舉中有速度之通過路口機率有改善
                        optimalSpeed = self.recommendSpeed
                        print("Plan B: OBU ID = %s +++++++++ Optimal speed = %d m/s ++++++++" % (self.OBU_ID, optimalSpeed))
                        traci.vehicle.setSpeed(self.OBU_ID, optimalSpeed)
                        self.OBU_RECOMMENDED_SPEED_INDICATOR = 0 # 已經計算過建議速度，將功能關閉
                    else: # if self.recommendSpeed == 0

                        # 相較於原始速度之通過機率，窮舉後所有速度之通過機率皆沒有改善
                        print("Plan C: 所有建議速度均沒有比原速好，採用緩慢減速機制")\

                        busCurTime = (traci.simulation.getTime() - RSUs['I1'].CycleAccumulated)
                        targetPhaseStartTime = RSUs['I1'].plan[0].phases[self.targetPhase].startTime

                        if (targetPhaseStartTime > busCurTime): #時相尚未開始
                            # 時間差t = 目標時相起始時間 - (當下模擬絕對時間 - 週期累計時間)
                            t = targetPhaseStartTime - busCurTime
                            print("targetPhaseStartTime > busCurTime / t = ", t)
                        else: # targetPhaseStartTime <= busCurTime
                            targetPhaseStartTime = RSUs['I1'].plan[1].phases[self.targetPhase].startTime
                            t = targetPhaseStartTime - busCurTime
                            print("targetPhaseStartTime <= busCurTime / t = ", t)

                        if self.direction in ['East', 'West']:  # 東西向
                            dist = round(abs(RSUs['I1'].location[0] - self.position[0]))  # 計算到路口的距離
                        elif self.direction in ['Nort', 'Sout']:  # 南北向
                            dist = round(abs(self.position[0]))  # 計算到路口的距離

                        self.acceleration = (dist - (self.currentSpeed * t * 2)) / t**2
                        print("dist = %d /  t = %d  / currentSpeed = %d / acceleration = %f" % (dist, t, self.currentSpeed , self.acceleration))
                        self.PLAN_C_INDICATOR = True  #開啟planC indicator
                        self.OBU_RECOMMENDED_SPEED_INDICATOR = 0 # 已經計算過建議速度，將功能關閉

                else:
                    print("OBU_RECOMMENDED_SPEED_INDICATOR = %d / self.currentSpeed = %d" % (self.OBU_RECOMMENDED_SPEED_INDICATOR, self.currentSpeed))
                    print("速度建議功能已被關閉")
            else: # if (result == False)
                # print("self.findTargetIntersections() return %s" % result)
                # print("OBU: %s 公車即將離開路網，重新開啟速度建議功能，將速度控制權還給SUMO" % self.OBU_ID)
                # # 因不確定是否車輛還有受速度控制，統一下指令將控制權還給SUMO
                # traci.vehicle.setSpeed(self.OBU_ID, -1)  # 引數-1 表示將控制權還給SUMO
                # self.OBU_RECOMMENDED_SPEED_INDICATOR = 1 # 重新開啟速度建議功能
                # self.PLAN_C_INDICATOR = False #將plan C indicator 關閉
                pass
        else:
            print("OBU_START_INDICATOR != 1  沒有開啟OBU功能")

        # 逐秒執行 plan C
        if (self.PLAN_C_INDICATOR == True):
            self.currentSpeed = self.currentSpeed + self.acceleration #Expected 減速度 = negative value
            print("Plan C: self.acceleration = %d / self.currentSpeed = %d" % (self.acceleration, self.currentSpeed))
            if (self.currentSpeed < 6): #不能讓車輛在路中停止，需要有個最小速度維持
                traci.vehicle.setSpeed(self.OBU_ID, 6)
            else:
                traci.vehicle.setSpeed(self.OBU_ID, self.currentSpeed)
        else:
            # print("no plan C executing")
            pass

    # Override
    def calOptimalSpeed(self, RSUs): #輸入路口RSU物件
        # RSUs = {'I1':I1(RSU物件), 'I2':I2(RSU物件), ... }
        # 最大通過機率預設為 = 通過機率限制
        maxPassProb = self.passProbThreshold
        print("self.needOptIntersectionList = ", self.needOptIntersectionList)
        # for i in self.needOptIntersectionList: # 累計各路口通過機率
        #     maxPassProb = maxPassProb + self.intersectionPassProb[i]

        # 計算各速度下的通過機率
        maxSpeed = round((1 + self.MaxSpeedFactor) * self.maxSpeedLimit)  # 設定建議速度上限
        minSpeed = round((1 - self.MinSpeedFactor) * self.maxSpeedLimit)  # 設定建議速度下限
        originalSpeed = self.currentSpeed
        for v in range(minSpeed, maxSpeed):  # 從 minSpeed 到 maxSpeed 窮舉
            self.currentSpeed = v  # 將目前速度設為v

            for i in self.needOptIntersectionList:
                newPassProb = self.calPassProb(RSUs[i])  # 以此速度計算路口通過機率
                self.intersectionPassProb.update({i: newPassProb})  # 更新路口i的通過機率 intersectionPassProb list

             # 加總各路口通過機率成為totalPassProb
            totalPassProb = 0
            for i in self.needOptIntersectionList:
                totalPassProb = totalPassProb + self.intersectionPassProb[i]

            if (totalPassProb > maxPassProb):  # 若新計算結果大於舊的，則取代之
                print("速度 %d 之 路口通過總機率為 %f 比原本 %f 好" % (v, totalPassProb, maxPassProb))
                maxPassProb = totalPassProb
                self.recommendSpeed = v  # 新速度v作為建議速度
                print("OBU: %s 建議速度 = %d" % (self.OBU_ID, self.recommendSpeed))
            else:
                print("速度 %d 之 路口通過機率為 %f 沒有比原本 %f 好" % (v, totalPassProb, maxPassProb))
                self.currentSpeed = originalSpeed #將速度改回原始速度


    # Override
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
            #print("公車方向: 向東")
            # 3. 找出targetIntersection
            for i in range(startIntersectionIndex, len(intersectionList)):  # 從群組之起始路口開始往後算，列舉出每個路口即為target Intersection
                i_Str = intersectionList[i]  # 取出路口str型態名稱 ex. I1 I2
                self.targetIntersectionList.append(i_Str)  # 將路口列入targetIntersectionList
                self.intersectionPassProb.update({i_Str: 1})  # 更新路口加入通過機率計算(預設機率=1)

            return True

        elif self.direction == 'West':
            #print("公車方向: 向西")
            for i in range(startIntersectionIndex, -1, -1):
                i_Str = intersectionList[i]  # 取出路口str型態名稱 ex. I1 I2
                self.targetIntersectionList.append(i_Str)  # 將路口列入targetIntersectionList
                self.intersectionPassProb.update({i_Str: 1})  # 將路口加入通過機率計算(預設機率=1)

            return True

        elif (self.direction == 'Nort' or self.direction == 'Sout'):  # 因為veh type擷取只有前4位字元
            #print("公車方向: 向北 / 向南")
            i_Str = self.nextTLS
            self.targetIntersectionList.append(i_Str)
            self.intersectionPassProb.update({i_Str: 1})  # 將路口加入通過機率計算(預設機率=1)
            # # self.position[1] = -200 -> -188 -> -165 ...
            # dist = round(abs(self.position[1]))  # self.position[1] 取y座標並轉換為整數型態
            # passProb_of_i(self, RSUs[i_Str], dist)
            return True
        else:
            print("例外錯誤! 公車方向不屬於任何一方")
            return False

    def __str__(self):
        return 'OBU(OBU_ID = {0}, vehType = {1}, position  = {2}, currentSpeed = {3}, direction = {4}, nextTLS = {5}, targetPhase = {6}, recommendSpeed = {7})'\
            .format(self.OBU_ID, self.vehType, self.position , self.currentSpeed, self.direction, self.nextTLS, self.targetPhase, self.recommendSpeed)



