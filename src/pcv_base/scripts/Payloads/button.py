#!/usr/bin/env python
import rospy

def button(PayloadClass):
    class buttonClass(PayloadClass):
        def __init__(self):
            super().__init__()
            self.ser_send('$S0&')
            self.buttonState = 0
            
        def execute(self):
            super().execute()
            self.__rx_cycle_inst = rospy.Timer(rospy.Duration(0.2), self.__cycle)
            
        def shutdown(self):
            super().shutdown()
            self.buttonStop()
            self.__rx_cycle_inst.shutdown()
            
        def buttonStart(self):
            self.ser_send('$S1&')
            self.buttonState = 1
        
        def buttonStop(self):
            self.ser_send('$S0&')
            self.buttonState = 0
        
        def buttonPause(self):
            self.ser_send('$S3&')
            self.buttonState = 3
            
        def __cycle(self):
            state = self.ser_recv('@S','%',1)
            self.buttonState = int(state)
    return buttonClass
    
if __name__ == '__main__':
    pass
    
