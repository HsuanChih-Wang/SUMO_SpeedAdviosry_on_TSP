import traci

phaseStrDict = {0: "J1", 1: "J2", 2: "J3", 3: "J4",4: "J5", 5: "J6", 6: "J7", 7: "J8"}
phaseStrDict_rev = {"J1": 0, "J2":1, "J3":2, "J4":3, "J5":4, "J6":5, "J7":6, "J8":7}
TLS_index_pair = {0: {"phase": 'J1', "direction": 'South'}, 1: {"phase": 'J1', "direction": 'South'}, 2: {"phase": 'J1', "direction": 'South'},
                  3: {"phase": 'J6', "direction": 'South'}, 4: {"phase": 'J6', "direction": 'South'},
                  5: {"phase": 'J7', "direction": 'West'}, 6: {"phase": 'J7', "direction": 'West'}, 7: {"phase": 'J7', "direction": 'West'},
                  8: {"phase": 'J4', "direction": 'West'}, 9: {"phase": 'J4', "direction": 'West'},
                  10: {"phase": 'J5', "direction": 'North'}, 11: {"phase": 'J5', "direction": 'North'}, 12: {"phase": 'J5', "direction": 'North'},
                  13: {"phase": 'J2', "direction": 'North'}, 14: {"phase": 'J2', "direction": 'North'},
                  15: {"phase": 'J3', "direction": 'East'} ,16: {"phase": 'J3', "direction": 'East'} ,17:{"phase": 'J3', "direction": 'East'},
                  18: {"phase": 'J8', "direction": 'East'}, 19: {"phase": 'J8', "direction": 'East'}}

# free flow speed
Vf = 13
# 參數
ALPHA = 1
OCC_BUS = 30
OCC_AUTO = 1.5

class RSU:

    def __init__(self,ID,location,detectionRange):
        self.RSU_ID = ID
        self.location = location
        self.detectionRange = detectionRange
        self.CycleAccumulated = 0
        self.currentState = 0
        # 停等車輛
        self.vehQueue = {"J1": [], "J2": [], "J3": [], "J4": [], "J5": [], "J6": [], "J7": [], "J8": []}
        # 非停等車輛
        self.vehNoneQueue = {"J1": [], "J2": [], "J3": [], "J4": [], "J5": [], "J6": [], "J7": [], "J8": []}
        self.plan = []  # plan[0] = adaptivePlan / plan[1] = backgroundPlan

    ##################  Subfunctions ####################

    def getVehicleParameters(self, vehID):

        try:
            nextTLSID = traci.vehicle.getNextTLS(vehID)[0][0]  # String: 'I1', 'I2', 'I3'....
            nextTLSIndex = traci.vehicle.getNextTLS(vehID)[0][1]  # Int
            dist = traci.vehicle.getNextTLS(vehID)[0][2]  # distance

        except IndexError as error:
            # print("IndexError: ",error)
            nextTLSID = None
            nextTLSIndex = 99999
            dist = 99999

        vehType = traci.vehicle.getTypeID(vehID)

        if (nextTLSID == self.RSU_ID and (dist <= self.detectionRange or vehType in ["Bus","Special"] )): # 確認車輛前方路口屬於正確RSU
            # 兩條件可以加入計算: (1) 車輛離路口距離小於偵測範圍 或 (2) 車種是公車或特殊車輛
            activation = 'yes'
            TPxj = dist / Vf + traci.simulation.getTime() - self.CycleAccumulated + 2
            phase = TLS_index_pair[nextTLSIndex]['phase']
            direction =  TLS_index_pair[nextTLSIndex]['direction']

            if (vehType == 'Bus'):
                print("phase = ",phase)
                print("direction = ",direction)


        else:
            activation = 'no'
            TPxj = None
            phase = None
            direction = 'None'

        order = 9999  # 暫時給予，在 sortQueueList 中再進行編號
        vehSpeed = traci.vehicle.getSpeed(vehID)
        position = traci.vehicle.getPosition(vehID)

        # 判斷車輛是 queue 或 non-queue
        if traci.vehicle.getWaitingTime(vehID) > 1:
            isQueue = True
        else:
            isQueue = False

        if vehType == "Bus":  # 設定乘載人數
            occupancy = OCC_BUS
        elif vehType == "Special":
            occupancy = ALPHA * OCC_AUTO
        else:
            occupancy = OCC_AUTO

        vehPara = {"vehID": vehID, "Activation": activation, "order": order, "type": vehType, "isQueue": isQueue, "occupancy": occupancy,
                   "vehSpeed": vehSpeed, "nextTLSID": nextTLSID, "phase": phase, "dist": dist, "position": position,
                   "direction": direction, "ProjectArrTime(TPxj)": TPxj}
        return vehPara

    def __setQueue(self, vehX):
        # 1. 確認是沒有離開路口的車輛 (若是已經離開路口車輛則不處理)
        # 2. 依照該車輛phase帶入
        # 3. 確認車輛狀態(vehX['isQueue'])分別將 vehX 存入queue和none-queue
        if (vehX['nextTLSID'] != None):  # 確認是沒有離開路口的車輛
            if (vehX['phase'] == "J1"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J1"].append(vehX)
                else:
                    self.vehNoneQueue["J1"].append(vehX)
            elif (vehX['phase'] == "J2"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J2"].append(vehX)
                else:
                    self.vehNoneQueue["J2"].append(vehX)
            elif (vehX['phase'] == "J3"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J3"].append(vehX)
                else:
                    self.vehNoneQueue["J3"].append(vehX)
            elif (vehX['phase'] == "J4"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J4"].append(vehX)
                else:
                    self.vehNoneQueue["J4"].append(vehX)
            elif (vehX['phase'] == "J5"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J5"].append(vehX)
                else:
                    self.vehNoneQueue["J5"].append(vehX)
            elif (vehX['phase'] == "J6"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J6"].append(vehX)
                else:
                    self.vehNoneQueue["J6"].append(vehX)
            elif (vehX['phase'] == "J7"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J7"].append(vehX)
                else:
                    self.vehNoneQueue["J7"].append(vehX)
            elif (vehX['phase'] == "J8"):
                if vehX['isQueue'] == True:
                    self.vehQueue["J8"].append(vehX)
                else:
                    self.vehNoneQueue["J8"].append(vehX)
        else:
            # print("")
            pass
            # print("vehX", vehX, " 已離開路口")

    def __sortQueueList(self):

        for J in self.vehQueue:  # vehQueue = {"J1": [vehX, vehX, vehX], "J2": [] ...}
            self.vehQueue[J].sort(key=lambda s: s['dist'])  # 依照離路口距離排序
            # print("len(vehQueue[%s]) = %d" % (J, len(vehQueue[J])))
            for index in range(len(self.vehQueue[J])):  # 填入對應的順序編號
                self.vehQueue[J][index]['order'] = index + 1  # 從1開始編號
                # print(" vehQueue[%s][%d]['order'] = %d" % (J, index, vehQueue[J][index]['order']))

        for J in self.vehNoneQueue:  # vehNoneQueue = {"J1": [], "J2": [] ...}
            self.vehNoneQueue[J].sort(key=lambda s: s['dist'])  # 依照離路口距離排序
            # print("len(vehNoneQueue[%s]) = %d" % (J, len(vehNoneQueue[J])))
            for index in range(len(self.vehNoneQueue[J])):  # 填入對應的順序編號
                self.vehNoneQueue[J][index]['order'] = len(self.vehQueue[J]) + index + 1
                # print(" vehNoneQueue[%s][%d]['order'] = %d" % (J, len(vehQueue[J]) + index + 1, vehNoneQueue[J][index]['order']))

    def cleanQueueList(self):
        for J in self.vehNoneQueue:
            self.vehNoneQueue[J].clear()
        for J in self.vehQueue:
            self.vehQueue[J].clear()

    def start(self):
        vehIDlist = traci.vehicle.getIDList()  # 取出路網中所有車輛ID

        self.cleanQueueList() # 清除原本的queueList

        for veh in vehIDlist:
            vehX = self.getVehicleParameters(vehID=veh)
            if (vehX['Activation'] == 'yes'):
                print("vehX = ",vehX)
                self.__setQueue(vehX)
            else:
                #print("不在偵測範圍 vehX = ", vehX)
                pass

        self.__sortQueueList()  ## 排序Queuelist 和 noneQueuelist: 將車輛依離路口距離編號(order x)


    ################################################################


    def setPlan(self,plan):
        self.plan = plan

    def setID(self,ID):
        self.ID = ID

    def setLoaction(self,location):
        self.location = location

    def getCurrentState(self):
        self.currentState = traci.trafficlight.getRedYellowGreenState(self.RSU_ID)
        return self.currentState

    def calPhaseTimeSplit(self, targetPhase):

        # 引數說明：
        # 1. targetIntersection: 要計算的路口編號(str)
        # 2. plan: 傳入該路口完整計畫內容(plan)(包含k和k+1週期)
        # 3. targetPhase: 指定要計算的phase編號(str)
        # 路口完整計畫內容(plan)支援格式: [{j:Phase() for j in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8']},
        #                           {j:Phase() for j in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'J8']}]

        phaseTimeSplit = []  # 紀錄時相切割時間點

        # 確認時相是否為起頭時相
        if targetPhase in ['J1', 'J5']:  # 時相編號為1或5是週期起頭時相
            IsHeadPhase = True
            # 起頭時相 phaseSplit = [(0) 紅燈起始, (1) 紅燈結束, (2)紅燈起始,..., (n)紅燈結束]

            #### 計算週期K ####
            # Expect: plan[0][targetPhase].startTime = 0
            targetPhaseEndTime = self.plan[0].phases[targetPhase].startTime + self.plan[0].phases[targetPhase].green
            phaseTimeSplit.append(targetPhaseEndTime)  # 加入 時相結束時間 = (0) 紅燈起始

            #### 計算週期K+n ####
            # Expect: cycle = 57
            cycle = self.plan[1].cycle
            # cycle = (plan[1]['J4'].startTime + plan[1]['J4'].green + plan[1]['J4'].yellow + plan[1]['J4'].allRed) -  (plan[1]['J1'].startTime)  # 計算k+1週期長度

            for num in range(1, 4):  # 這裡修改可設定一次產生幾個週期(k+1 k+2 k+3...)後的時間
                if (num == 1):
                    phaseTimeSplit.append(self.plan[1].phases[targetPhase].startTime)  # 時相在週期k+1的起始時間 = (1) 紅燈結束
                    startPoint = self.plan[1].phases[targetPhase].startTime
                phaseTimeSplit.append(startPoint + self.plan[1].phases[targetPhase].green)  # 時相在週期k+n的結束時間 = (2) 紅燈起始
                startPoint = startPoint + cycle  # 再加一個週期 -> 時相在k+n的起始時間 = (n) 紅燈結束
                phaseTimeSplit.append(startPoint)

        elif targetPhase in ['J2', 'J3', 'J4', 'J6', 'J7', 'J8']:  # 其他時相編號 (非起頭時相)
            IsHeadPhase = False
            phaseTimeSplit.append(0)  # 在最開始處新增0 (表示由0秒處紅燈起始)
            # 非起頭時相 phaseSplit = [(0) 0, (1) 紅燈結束, (2) 紅燈起始, (3)紅燈結束, (4)紅燈起始, ... , (n)紅燈結束]
            #### 計算週期K ####0
            startPoint = self.plan[0].phases[targetPhase].startTime  # targetPhase的起始時間 ( = (1) 紅燈結束時間)
            phaseTimeSplit.append(startPoint)
            phaseTimeSplit.append(startPoint + self.plan[0].phases[targetPhase].green)  # targetPhase結束時間 ( = (2) 紅燈起始時間)

            #### 計算週期K+n ####
            # 計算k+1週期長度
            cycle = self.plan[1].cycle
            # cycle = (plan[1]['J4'].startTime + plan[1]['J4'].green + plan[1]['J4'].yellow + plan[1]['J4'].allRed) - (plan[1]['J1'].startTime)

            for num in range(1, 4):  # 這裡修改可設定一次產生幾個週期(k+1 k+2 k+3...)後的時間
                if (num == 1):
                    startPoint = self.plan[1].phases[targetPhase].startTime  # k+1週期的targetPhase起始時間 = (3) 紅燈結束
                    phaseTimeSplit.append(startPoint)
                phaseTimeSplit.append(startPoint + self.plan[1].phases[targetPhase].green)  # k+1週期的targetPhase結束時間 = (4) 紅燈起始
                startPoint = startPoint + cycle  # k+2週期targetPhase的起始時間 = (n) 紅燈結束
                phaseTimeSplit.append(startPoint)
        else:  # 例外錯誤
            print("例外錯誤:　targetPhase = ", targetPhase)

        return phaseTimeSplit, IsHeadPhase  # 回傳 [0] 時間分割點 [1] 是否為起頭時相

    def __str__(self):
        return 'RSU(ID = {0}, location = {1},  plan = {2})'\
            .format(self.RSU_ID, self.location, self.plan)





