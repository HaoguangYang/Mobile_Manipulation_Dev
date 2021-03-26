#!/usr/bin/env python
import rospy
import time

def uvc(PayloadClass):
    class uvcClass(PayloadClass):
        def __init__(self):
            super(uvcClass, self).__init__()
            self.ser_send('$L0&')
            self.uvcOn = False
            self.uvcCurrent = 0.0  # (A)
            self.__lowCurrentFlag = 0
            self.__last_notify = 0
            self.__notify_timeout = 3600
            #try:
            #    thread.start_new_thread( self.readSerial )
            #except:
            #    print "Error: unable to start Serial Listener thread"
            
        def execute(self):
            super(uvcClass, self).execute()
            self.__rx_cycle_inst = rospy.Timer(rospy.Duration(1.), self.__cycle)

        def turnOnUVC(self):
            self.ser_send('$L1&')
            self.uvcOn = True
            
        def turnOffUVC(self):
            self.ser_send('$L0&')
            self.uvcOn = False
            
        def shutdown(self):
            super(uvcClass, self).shutdown()
            self.turnOffUVC()
            self.__rx_cycle_inst.shutdown()
            
        def __cycle(self):
            current = self.ser_recv('@A','%',4)
            self.uvcCurrent = (float(current)-512.0)/4.83   # analog conversion of lamp current
            
            # should be in the evaluator.
            if self.uvcOn and self.uvcCurrent < 12.0:
                # delay, send sms until 20 consecutive low measurements.
                #print(lowCurrentFlag)
                self.__lowCurrentFlag += 1
            elif self.uvcOn and self.uvcCurrent >= 12.0:
                self.__lowCurrentFlag = 0
            else:
                pass

            
            if self.__lowCurrentFlag > 20:
                timeNow = time.time()
                if timeNow - self.__last_notify > self.__notify_timeout :
                    # TODO: notification migrated to a separate util code parallel with database logger. 
                    # Publish to a notification topic as text to get handeled.
                    # send text msg
                    """
                    message = self.sms_client.messages \
                              .create(
                                    body='FIXME: UVC Lamp Malfunction, Battery is Low or Lamp is Broken!',
                                    from_=self.sms_from, # this is my twilio number
                                    to=self.sms_to # this is my number
                              )
                    """
                    print('FIXME: UVC Lamp Malfunction, Battery is Low or Lamp is Broken!')
                    print('message sent!')
                    self.__last_notify = timeNow

    return uvcClass
    
if __name__ == '__main__':
    pass
    
