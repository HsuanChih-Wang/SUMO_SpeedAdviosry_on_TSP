import traci

ring1_state = ['GGGrrrrrrrrrrrrrrrrr','yyyrrrrrrrrrrrrrrrrr','rrrrrrrrrrrrrrrrrrrr', #0,1,2
                'rrrrrrrrrrrrrGGrrrrr','rrrrrrrrrrrrryyrrrrr','rrrrrrrrrrrrrrrrrrrr', #13,14
                'rrrrrrrrrrrrrrrGGGrr','rrrrrrrrrrrrrrryyyrr','rrrrrrrrrrrrrrrrrrrr', #15,16,17
                'rrrrrrrrGGrrrrrrrrrr','rrrrrrrryyrrrrrrrrrr','rrrrrrrrrrrrrrrrrrrr'] #8,9
ring2_state = ['rrrrrrrrrrGGGrrrrrrr','rrrrrrrrrryyyrrrrrrr','rrrrrrrrrrrrrrrrrrrr', #10,11,12
                'rrrGGrrrrrrrrrrrrrrr','rrryyrrrrrrrrrrrrrrr','rrrrrrrrrrrrrrrrrrrr', #3,4
                'rrrrrGGGrrrrrrrrrrrr','rrrrryyyrrrrrrrrrrrr','rrrrrrrrrrrrrrrrrrrr', #5,6,7
                'rrrrrrrrrrrrrrrrrrGG','rrrrrrrrrrrrrrrrrryy','rrrrrrrrrrrrrrrrrrrr'] #18,19

phaseAndGreenStatePairs = {"J1": [0,1,2], "J2": [13,14], "J3": [15,16,17], "J4": [8,9], "J5": [10,11,12], "J6": [3,4], "J7": [5,6,7], "J8": [18,19]}
phaseStrDict = {0: "J1", 1: "J2", 2: "J3", 3: "J4",4: "J5", 5: "J6", 6: "J7", 7: "J8"}
phaseStrDict_rev = {"J1": 0, "J2": 1, "J3": 2, "J4": 3, "J5": 4, "J6": 5, "J7": 6, "J8": 7}


def makeLogicObject(currentPhase, tijkResult, phasePendingStatus):
    # expect: currentPhase = [0,4] / tijkResult = [1,4,14,7,0,5,17,4]
    # phasePendingStatus = {0: R, 1: R, 2: R, 3: R,
    #                       4: R, 5: R, 6: R, 7: R}

    num_Of_G = []
    num_Of_y = []
    num_Of_r = []
    for phase in currentPhase:
        if (phasePendingStatus[phase] in [5]):
            phaseStr = phaseStrDict[phase]  # 轉成字串形式
            if (tijkResult[phase] > 0):
                for num in phaseAndGreenStatePairs[phaseStr]:
                    num_Of_G.append(num)
            else:  # tijkResult[phase] = 0
                for num in phaseAndGreenStatePairs[phaseStr]:
                    num_Of_y.append(num)
        elif (phasePendingStatus[phase] in [3, 4]):
            if phase == 0:
                previousPhase = 3
            elif phase == 4:
                previousPhase = 7
            else:
                previousPhase = phase - 1
            phaseStr = phaseStrDict[previousPhase]  # 這個phasePending -> 代表前一個phase還在清道時間，要設定前一個phase的state
            for num in phaseAndGreenStatePairs[phaseStr]:
                num_Of_y.append(num)
        elif (phasePendingStatus[phase] in [1, 2]):
            if phase == 0:
                previousPhase = 3
            elif phase == 4:
                previousPhase = 7
            else:
                previousPhase = phase - 1
            phaseStr = phaseStrDict[previousPhase]  # 這個phasePending -> 代表前一個phase還在清道時間，要設定前一個phase的state
            for num in phaseAndGreenStatePairs[phaseStr]:
                num_Of_r.append(num)
        else:
            print("例外錯誤: phasePendingStstus 輸入不合預期")

    # 2.產生state
    state = ''
    for item in range(0, 20):
        if item in num_Of_G:
            state = state + 'G'
        elif item in num_Of_y:
            state = state + 'y'
        elif item in num_Of_r:
            state = state + 'r'
        else:
            state = state + 'r'
    print("state = ", state)

    phaseResult = []
    phaseResult.append(traci.trafficlight.Phase(duration=1, state=state, minDur=1, maxDur=1, next=()))

    # 一組program就是一個logic物件
    LogicObject = traci.trafficlight.Logic(programID='1', type=0, currentPhaseIndex=0,
                                           phases=phaseResult)
    print("LogicObject = ", LogicObject)
    return LogicObject

class SignalPlan:

    def setAllParameters(self,planID,order,cycle,offset,phases=[]):
        # RSUid,location,BGplan,ADplan
        #super(RSUid=RSUid,location=location,backgroundplan=BGplan,adaptiveplan=ADplan)
        self.planID = planID
        self.planOrder = order # ex. B0 00 01...
        self.cycle = cycle
        self.offset = offset
        self.phases = phases

    def __str__(self):
        return 'SignalPlan(planID = {0}, planOrder = {1}, cycle = {2}, offset = {3}, phases = {4})'\
            .format(self.planID, self.planOrder, self.cycle, self.offset, self.phases)

    def OLDmakeLogicObject(self):

        def splitDualRingtoTLS(phases):
            # 接受phase物件
            # ex. phases = {"J1": Phase(order = 0, name = 北往南直右, startTime = 0, green = 10, yellow = 3, allRed = 2), ..., }

            def splitRingTime(RING):
                ringIndex = 0
                combined_state_index = 0
                if RING == 'ring1':
                    for item in ringSplitTime:
                        if (item < ring1[ringIndex]):
                            combined_state[RING].append(ring1_state[ringIndex])
                        elif (item == ring1[ringIndex]):
                            combined_state[RING].append(ring1_state[ringIndex])
                            ringIndex = ringIndex + 1
                        else:
                            ringIndex = ringIndex + 1
                            combined_state[RING].append(ring1_state[ringIndex])
                else:
                    for item in ringSplitTime:
                        if (item < ring2[ringIndex]):
                            combined_state[RING].append(ring2_state[ringIndex])
                        elif (item == ring2[ringIndex]):
                            combined_state[RING].append(ring2_state[ringIndex])
                            ringIndex = ringIndex + 1
                        else:
                            ringIndex = ringIndex + 1
                            # print("ring2_state = ", ring2_state)
                            # print("len  = ",len(ring2_state))
                            # print("ringIndex = ", ringIndex)
                            try:
                                combined_state[RING].append(ring2_state[ringIndex])
                            except IndexError as e:
                                print("抓!PhaseObject 有IndexError例外錯誤! 記得內審完後來修我!")
                                print(e)
                                return True

            ring1 = []
            ring2 = []
            for phase in phases:
                if (phase in ['J1', 'J2', 'J3', 'J4']):
                    try:
                        phaseList = [phases[phase].green,
                                     phases[phase].yellow, phases[phase].allRed]
                        ring1.extend(phaseList)
                    except AttributeError as error:
                        print("ring1. Attribute Error")

                else:  # phase in J5 J6 J7 J8
                    try:
                        phaseList = [phases[phase].green,
                                     phases[phase].yellow, phases[phase].allRed]
                        ring2.extend(phaseList)
                    except AttributeError as error:
                        print("ring2. Attribute Error")

            combined_state = {"ring1": [], "ring2": []}
            finalSignalPlan = {"state": [], "duration": []}

            for index in range(1, len(ring1)):
                ring1[index] = ring1[index - 1] + ring1[index]
                ring2[index] = ring2[index - 1] + ring2[index]

            ringSplitTime = ring1 + ring2
            ringSplitTime.sort()
            ringSplitTime = list(dict.fromkeys(ringSplitTime))  # 去掉重複的

            splitRingTime('ring1')
            splitRingTime('ring2')

            # 將ring1和ring2 結果合併
            for strIndex in range(len(ringSplitTime)):
                tempStr = ""

                if combined_state['ring1'][strIndex] != combined_state['ring2'][strIndex]:
                    for charIndex in range(20):  # len(phaseIndex) = 20
                        if (combined_state['ring1'][strIndex][charIndex] != 'r'):
                            tempStr = tempStr + combined_state['ring1'][strIndex][charIndex]
                        elif (combined_state['ring2'][strIndex][charIndex] != 'r'):
                            tempStr = tempStr + combined_state['ring2'][strIndex][charIndex]
                        else:
                            tempStr = tempStr + 'r'
                else:
                    tempStr = combined_state['ring1'][strIndex]

                finalSignalPlan['state'].append(tempStr)  # 將最終結果存入

            for index in range(len(ringSplitTime)):
                if (index > 0):
                    duration = ringSplitTime[index] - ringSplitTime[index - 1]
                else:
                    duration = ringSplitTime[index]
                finalSignalPlan['duration'].append(duration)

            # print("finalSignalPlan = ",finalSignalPlan)
            return finalSignalPlan

        #接受一個完整計畫內容:
        # IntersectionSignal[路口編號][週期編號] = {"J1":J1(Phase物件, 以此類推), "J2":J2, "J3":J3, "J4":J4, "J5":J5, "J6":J6, "J7":J7, "J8":J8}
        TLS_result = splitDualRingtoTLS(self.phases)
        print("TLS_result = ", TLS_result)

        phaseResult = []
        for index in range(len(TLS_result['state'])):
            STATE = TLS_result['state'][index]
            DURATION = TLS_result['duration'][index]
            phaseResult.append(
                traci.trafficlight.Phase(duration=DURATION, state=STATE, minDur=DURATION, maxDur=DURATION, next=()))

        # 一組program就是一個logic物件
        LogicObject = traci.trafficlight.Logic(programID='1', type=0, currentPhaseIndex=0, phases=phaseResult) #phases不一定要tuple

        return LogicObject


class Phase(SignalPlan):

    def __init__(self, phaseID, phaseOrder, startTime, green, Gmin, Gmax, yellow, allRed):
        phaseOrderName = ["北往南直右", "南往北左轉", "西往東直右", "東往西左轉",
                          "南往北直右", "北往南左轉", "東往西直右", "西往東左轉", ]  # 8個分相
        self.phaseID = phaseID
        self.phaseOrder = phaseOrder
        self.name = phaseOrderName[phaseOrder]
        self.startTime = startTime
        self.green = green
        self.Gmin = Gmin
        self.Gmax = Gmax
        self.yellow = yellow
        self.allRed = allRed

    def setStartTime(self, startTime):
        self.startTime = startTime

    def setPhaseGreen(self, green):
        self.green = green

    def setClearenceTime(self, yellow, allRed):
        self.yellow = yellow
        self.allRed = allRed

    def __str__(self):
        return 'Phase(order = {0}, name = {1}, startTime = {2}, green = {3}, yellow = {4}, allRed = {5})' \
            .format(self.phaseOrder, self.name, self.startTime, self.green, self.yellow, self.allRed)









