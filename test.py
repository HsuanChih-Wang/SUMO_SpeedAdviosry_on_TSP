
class vehInfo:
    def __init__(self, vehID, startTime, endTime):
        self.vehID = vehID
        self.startTime = startTime
        self.endTime = endTime

    def __str__(self):
        return 'vehID = {0} startTime = {1} endTime = {2}'.format(self.vehID, self.startTime, self.endTime)


veh01 = vehInfo("BUS01", "0", "60")


vehInfo = {}
print(veh01)