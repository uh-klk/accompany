import datetime
from Data.dataAccess import DataAccess
from threading import Thread
from extensions import PollingThread, PollingProcessor

class ActionHistory(object):
    _defaultImageType = 'png'
    _runningThreads = {}
    
    def cancelPollingHistory(self, ruleName):
        if ActionHistory._runningThreads.has_key(ruleName):
            ah = ActionHistory._runningThreads[ruleName]
            ah.cancel()
            ah.join()
            return True
        else:
            return False

    def addPollingHistory(self, ruleName, delaySeconds):
        if not ActionHistory._runningThreads.has_key(ruleName):
            ahw = PollingThread(target=self.addHistory, delayTime=delaySeconds, args=(ruleName,), completeCallback=self._removePollingHistory)
            ahw.start()
            ActionHistory._runningThreads[ruleName] = ahw
        
        return ruleName
    
    def _removePollingHistory(self, ruleName):
        return ActionHistory._runningThreads.pop(ruleName, None)

    def addHistoryAsync(self, ruleName, imageBytes=None, imageType=None):
        Thread(target=self.addHistory, args=(ruleName, imageBytes, imageType)).start()

    def addHistory(self, ruleName, imageBytes=None, imageType=None):
        
        from Robots.robotFactory import Factory
        cob = Factory.getCurrentRobot()
        dao = DataAccess()
        dateNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        location = dao.getRobotByName(cob.name)['locationId']
        
        historyId = dao.saveHistory(dateNow, ruleName, location)
        
        if(historyId > 0):
            dao.saveSensorHistory(historyId)

            if imageType == None:
                imageType = ActionHistory._defaultImageType

            if imageBytes == None:
                imageBytes = cob.getImage(retFormat=imageType)

            if imageBytes != None:
                dao.saveHistoryImage(historyId, imageBytes, imageType)
        
        return historyId > 0

################################################################################
#
# Logger thread
#
# Logs channel value and / or status changes into a (separate) MySQL
# database table.
#
################################################################################
class SensorLog(PollingProcessor):    
    def __init__ (self, channels, name=''):
        super(SensorLog, self).__init__()
        self._dao = DataAccess().sensors
        self._channels = channels
        self._logCache = {}
        self._name = name
                
    def start(self):
        if self._name != '':
            print "Started updating database for %s sensor changes" % (self._name)
        else:
            print "Started updating database for [unknown] sensor changes"
        self._addPollingProcessor('sensorHistory', self.checkUpdateSensors, (self._channels,), 0.01)

    def stop(self):
        if self._name != '':
            print "Stopped updating database for %s sensor changes" % (self._name)
        else:
            print "Stopped updating database for [unknown] sensor changes"

        self._removePollingProcessor('sensorHistory')

    def checkUpdateSensors(self, channels):
        for uuid, sensor in channels.items():
            if not self._logCache.has_key(uuid):
                current = self._dao.getSensor(sensor['id'])
                self._logCache[uuid] = { 'value': current['value'], 'status': current['status']}

            status = str(sensor['status']).capitalize()
            if self._logCache[uuid]['status'] != status or self._logCache[uuid]['value'] != sensor['value']:
                timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                success = self._dao.saveSensorLog(
                                                  sensor['id'],
                                                  sensor['value'],
                                                  status,
                                                  timestamp,
                                                  sensor['room'],
                                                  sensor['channel'])
                if success:
                    print "Updated sensor log for %(id)s to %(status)s (%(value)s)" % { 
                                                                           'id':sensor['channel'],
                                                                           'status': status,
                                                                           'value': sensor['value'],
                                                                           }
                    self._logCache[uuid]['value'] = sensor['value']
                    self._logCache[uuid]['status'] = status

if __name__ == '__main__':
    import sys
    print >> sys.stderr, "Sensor update code has moved to sensor.py"
    print >> sys.stderr, "Run 'python sensors.py' to begin monitoring sensors"