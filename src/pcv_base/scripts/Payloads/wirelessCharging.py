#!/usr/bin/env python
import rospy
import time

def wirelessCharger(PayloadClass):
    class wirelessChargerClass(PayloadClass):
        def __init__(self):
            super().__init__()
            self.ser_send('$C0&')
            self.chargingState = 0
            self.chargingError = False
            
        def execute(self):
            super().execute()
            self.__cycle_inst = rospy.Timer(rospy.Duration(1), self.__cycle)
            
        def shutdown(self):
            super().shutdown()
            self.chargingStop()
            self.__cycle_inst.shutdown()
            
        def chargingStart(self):
            self.ser_send('$C1&')
            self.chargingState = 1
        
        def chargingStop(self):
            self.ser_send('$C0&')
            self.chargingState = 0
            
        def __cycle(self):
            self.ser_send('$CQ&')
            state = self.ser_recv('@CE','%',1)
            if state:
                self.chargingError = True
                self.chargingStop()
                time.sleep(0.5)
                self.chargingStart()
    
    return wirelessChargerClass
        
if __name__ == '__main__':
    pass
    
