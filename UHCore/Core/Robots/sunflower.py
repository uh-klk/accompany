import math
import time
import Data.dataAccess
import rosHelper

class Sunflower(object):
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name='Sunflower'):
        self._poseProcessor = None
        self._rs = None
        self._name = name

    @property
    def _ros(self):
        if self._rs == None:
            #Wait to configure/initROS ROS till it's actually needed
            rosHelper.ROS.configureROS(version='electric', packagePath=None, packageName=None, rosMaster='http://sf1-1-pc1:11311', overlayPath='/home/nathan/git/sunflower/sf_lights')
            self._rs = rosHelper.ROS()
        return self._rs

    @property
    def _pose(self):
        if self._poseProcessor == None:
            self._poseProcessor = rosHelper.Transform(self._ros)
        return self._poseProcessor
    
    @property
    def name(self):
        return self._name

    def getImage(self, leftRight='right', retFormat='PNG'):
        pass

    def executeFunction(self, funcName, kwargs):
        """Exectues an arbitrary function that the robot supports"""
        pass

    def getLocation(self, raw=False):        
        p = self._pose
        ((x, y, _), rxy) = p.getRobotPose()
        if x == None or y == None:
            if raw:
                return (None, None, None)
            else:
                return (None, (None, None, None))
        
        angle = round(math.degrees(rxy))
        pos = (round(x, 3), round(y, 3), angle)
        
        if raw:
            return pos
        else:
            return Data.dataAccess.Locations.resolveLocation(pos)

    def setLight(self, colour):
        try:
            self._ros.configureROS(packageName='sf_lights')
            import actionlib
            import sf_lights.msg
            
            goal = sf_lights.msg.LightsGoal(rgb=colour)            
            client = actionlib.SimpleActionClient('/lights', sf_lights.msg.LightsAction)
            client.wait_for_server()
            return client.send_goal_and_wait(goal)
        except:
            return 3
        pass

    def setComponentState(self, name, value):
        """set a component on the robot to the specified value"""
        """for example 'tray' 'up"""
        pass

    def getComponentPositions(self, componentName):
        """returns the list of available position(s) of a component"""
        """['up', 'down', etc]"""
        pass

    def getComponents(self):
        """gets a list of available component names ['tray', 'arm', etc]"""
        pass

    def getComponentState(self, componentName, raw=False):
        """gets the current component state, if raw==False, resolves the named state to one of the
           states returned from getComponentPositions(), or '' if not near to any named state"""
        pass

if __name__ == '__main__':
    s = Sunflower()
    print s.setLight([0,1,0])
