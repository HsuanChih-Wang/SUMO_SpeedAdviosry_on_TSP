import xml.etree.ElementTree as ET

f1 = 'thesis_single_priority_tripInfo1.xml'
f2 = 'thesis_single_priority_tripInfo2.xml'
f3 = 'thesis_single_priority_tripInfo3.xml'
f4 = 'thesis_single_priority_tripInfo4.xml'
f5 = 'thesis_single_priority_tripInfo5.xml'

f0_1 = 'thesis_single_noPriority_tripInfo_1.xml'
f0_2 = 'thesis_single_noPriority_tripInfo_2.xml'
f0_3 = 'thesis_single_noPriority_tripInfo_3.xml'
f0_4 = 'thesis_single_noPriority_tripInfo_4.xml'
f0_5 = 'thesis_single_noPriority_tripInfo_5.xml'

fileName0 = [f0_1, f0_2, f0_3, f0_4, f0_5]
fileName1 = [f1, f2, f3, f4, f5]
fileName = ['bus_operation_tripInfo.xml']
batteryName = ['bus_operation_Battery01.out.xml']

#for child in root:
    # print(child.tag, child.attrib)  # 印出tag和屬性

id_arterialCarFlow = ["Phase3", "Phase4", "Phase7", "Phase8"]
id_sideCarFlow = ["Phase1", "Phase2", "Phase5", "Phase6"]
id_BusFlow = ["Bus"]

#乘載人數
bus_Occupancy = 30
auto_Occupancy = 1.5

for file in fileName:

    tree = ET.ElementTree(file=file)
    root = tree.getroot()


    delay_Bus = 0  # 計算公車延滯
    delay_arterialCar = 0  # 計算幹道小汽車延滯
    delay_sideCar = 0  # 計算支道小汽車延滯

    numOfArterialCar = 0  # 計數幹道車數量
    numOfSideCar = 0 # 計數支道車數量
    numOfBus = 0  # 計數公車數量


    for tripinfo in root.findall('tripinfo'):  # 找tag叫做tripinfo者
        id = tripinfo.get('id')  # 取得標籤 id 的值
        waitingTime = tripinfo.get('waitingTime')  # 取得標籤 waitingTime 的值
        print("%s, %s" % (id, waitingTime))  # 印出兩者
        print(tripinfo.get('depart'))
        if float(tripinfo.get('depart')) >= 300 and float(tripinfo.get('depart')) <= 7200:
            for arterialCar in id_arterialCarFlow:
                if id.find(arterialCar) is not -1:
                    WaitingTime_arterialCar = waitingTime
                    print("WaitingTime_arterialCar=", WaitingTime_arterialCar)
                    delay_arterialCar = delay_arterialCar + float(WaitingTime_arterialCar)  # 累加一般車延滯
                    numOfArterialCar = numOfArterialCar + 1  # 幹道車數量+1
                    print("numOfArterialCar = ", numOfArterialCar)

            for sideCar in id_sideCarFlow:
                if id.find(sideCar) is not -1:
                    WaitingTime_sideCar = waitingTime
                    print("WaitingTime_sideCar=", WaitingTime_sideCar)
                    delay_sideCar = delay_sideCar + float(WaitingTime_sideCar)  # 累加一般車延滯
                    numOfSideCar = numOfSideCar + 1  # 支道車數量+1
                    print("numOfSideCar = ", numOfSideCar)

            for Bus in id_BusFlow:
                if id.find(Bus) is not -1:
                    WaitingTime_Bus = waitingTime
                    print("WaitingTime_Bus=", WaitingTime_Bus)
                    delay_Bus = delay_Bus + float(WaitingTime_Bus)  # 累加公車延滯
                    numOfBus = numOfBus + 1  # 公車數量+1
                    print("numOfBus = ", numOfBus)

    delay_allCar = delay_arterialCar + delay_sideCar + delay_Bus
    numOfTotalVehicle = numOfArterialCar + numOfSideCar + numOfBus
    # numOfTotalPassenger = (auto_Occupancy * numOfAuto) + (bus_Occupancy * numOfBus)

    D_TV = round((delay_Bus / numOfBus), 2)
    D_MV = round((delay_arterialCar / numOfArterialCar), 2)
    D_SV = round((delay_sideCar / numOfSideCar), 2)
    D_AV = round((delay_allCar / numOfTotalVehicle), 2)

    print("--- ", file, " --- ")
    print("------------------------------")
    print("delay_allCar = ", delay_allCar)
    print("delay_sideCar = ", delay_sideCar)
    print("delay_arterialCar = ", delay_arterialCar)
    print("delay_Bus = ", delay_Bus)

    print("numOfArterialCar = ", numOfArterialCar)
    print("numOfSideCar = ", numOfSideCar)
    print("numOfBus = ", numOfBus)
    print("numOfTotalVehicle = ", numOfTotalVehicle)
    #print("numOfTotalPassenger = ", numOfTotalPassenger)

    print("Average delay of transit vehicle (D-TV) =", D_TV)
    print("Average delay of main-street vehicle (D-MV) =", D_MV)
    print("Average delay of side-street vehicle (D-SV) =", D_SV)
    print("Average delay of all vehicle (D-AV) =", D_AV)


    D_TV_result = []
    D_MV_result = []
    D_SV_result = []
    D_AV_result = []

    D_TV_result.append(D_TV)
    D_MV_result.append(D_MV)
    D_SV_result.append(D_SV)
    D_AV_result.append(D_AV)

sD_TV_result = [str(a) for a in D_TV_result]
sD_MV_result = [str(a) for a in D_MV_result]
sD_SV_result = [str(a) for a in D_SV_result]
sD_AV_result = [str(a) for a in D_AV_result]

print(','.join(sD_TV_result))
print(','.join(sD_MV_result))
print(','.join(sD_SV_result))
print(','.join(sD_AV_result))


