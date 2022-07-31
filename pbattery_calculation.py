import xml.etree.ElementTree as ET

f1 = 'result/bus_operation_tripInfo_1.xml'
f2 = 'result/bus_operation_tripInfo_2.xml'
f3 = 'result/bus_operation_tripInfo_3.xml'
f4 = 'result/bus_operation_tripInfo_4.xml'
f5 = 'result/bus_operation_tripInfo_5.xml'

fileName1 = [f1, f2, f3, f4, f5]
fileName = ['bus_operation_tripInfo.xml']
batteryName = ['result/bus_operation_batteryOut_1.xml', 'result/bus_operation_batteryOut_2.xml',
               'result/bus_operation_batteryOut_3.xml', 'result/bus_operation_batteryOut_4.xml',
               'result/bus_operation_batteryOut_5.xml']

#for child in root:
    # print(child.tag, child.attrib)  # 印出tag和屬性

id_arterialCarFlow = ["Phase3", "Phase4", "Phase7", "Phase8"]
id_sideCarFlow = ["Phase1", "Phase2", "Phase5", "Phase6"]
id_BusFlow = ["Bus"]

# #乘載人數
# bus_Occupancy = 30
# auto_Occupancy = 1.5

avgEnergyConsumed_result = []
avgtotalAcceleration_result = []


for file in fileName:

    tree = ET.ElementTree(file=file)
    root = tree.getroot()
    totalEnergyConsumed = 0
    totalAcceleration = 0

    for vehicle in root.iter('vehicle'):
        if (vehicle.attrib.get('id')[0:3] == 'Bus'):
            energyConsumed = vehicle.attrib.get('energyConsumed')
            actualBatteryCapacity = vehicle.attrib.get('actualBatteryCapacity')
            acceleration = vehicle.attrib.get('acceleration')
            totalEnergyConsumed = float(totalEnergyConsumed) + float(energyConsumed)
            totalAcceleration = float(totalAcceleration) + abs(float(acceleration))

            #print("vehicle id = %s / vehicle energyConsumed = %s / actualBatteryCapacity  = %s / acceleration = %s " %(vehicle.attrib.get('id'), energyConsumed, actualBatteryCapacity, acceleration))


    print("totalEnergyConsumed = %f / average = %f" %(totalEnergyConsumed, totalEnergyConsumed / 12))
    print("totalAcceleration = %f / average = %f" % (totalAcceleration, totalAcceleration / 12))
    avgEnergyConsumed_result.append(round((totalEnergyConsumed/12), 2))
    avgtotalAcceleration_result.append(round((totalAcceleration/12), 2))

s_avgEnergyConsumed_result = [str(a) for a in avgEnergyConsumed_result]
s_avgtotalAcceleration_result = [str(a) for a in avgtotalAcceleration_result]

print(','.join(s_avgEnergyConsumed_result))
print(','.join(s_avgtotalAcceleration_result))


