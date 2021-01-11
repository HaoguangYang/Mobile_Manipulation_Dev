#!/usr/bin/env python

import serial
import time
import multiprocessing
#from twilio.rest import Client
import xml.etree.ElementTree as ET 

class genericPayload():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', timeout=2)  # open serial port.
        time.sleep(1)
        self.ser.setDTR(0)
        time.sleep(1)
        """
        sms_cred = ET.parse('/home/cartman/Dev/smsCredentials.xml')
        account_sid = sms_cred.findall('account_sid')[0].get('value')
        auth_token = sms_cred.findall('auth_token')[0].get('value')
        self.sms_client = Client(account_sid, auth_token)
        self.sms_from = sms_cred.findall('from')[0].get('value')
        self.sms_to = sms_cred.findall('to')[0].get('value')
        """
        self.mgr = multiprocessing.Manager()
        self.d = self.mgr.dict()
        self.d[0] = 0       # State: 0 - stopped; 3 - running; 5 - paused
        self.d[1] = False   # notified
        self.d[2] = 0       # queried value
        self.d[3] = False   # new value available
        self.d[4] = -1      # measurement to query
        self.d[5] = True    # rst_nCmd (Software-generated)
        #try:
        #    thread.start_new_thread( self.readSerial )
        #except:
        #    print "Error: unable to start Serial Listener thread"

    def serialComm(self):
        self.ser.write('$S0&')
        while 1:
            if self.ser.inWaiting():
                header = self.ser.read(1)           # read one byte
                if header == '@':                   # header captured
                    cmd = self.ser.read(1)          # read second byte     
                    if cmd == 'A':
                        meas = self.ser.read(4)
                        end = self.ser.read(1)
                        if end == '%':              # tail matches, valid msg.
                            self.d[2] = int(meas)   # save current analog measurement
                            self.d[3] = True
                    elif cmd == 'S':
                        status = self.ser.read(1)
                        end = self.ser.read(1)
                        if end == '%':              # tail matches, valid msg
                            self.d[0] = int(status)
                    else:
                        pass
                else:                               # not a message.
                    pass
            if self.d[4] >= 0:
                self.ser.write('$Q'+str(self.d[4])+'&')
                self.d[4] = -1
            if not self.d[1]:
                self.ser.write('$S'+str(self.d[0])+'&')
                self.d[1] = True
            if self.d[5] == False:
                self.ser.write('$S0&')
                self.d[0] = 0
                self.d[1] = True
                self.d[2] = 0
                self.d[3] = False
                self.d[4] = -1
                self.d[5] = True

    def initialize(self):
        self.ser.write('$S0&')
        self.d[0] = 0
        self.d[1] = True
        self.payload_thread = multiprocessing.Process(target=self.serialComm, args=())
        self.payload_thread.daemon = True
        self.payload_thread.start()
        
    def pause(self):
        self.d[0] = 5
        self.d[1] = False
    
    def resume(self):
        self.d[0] = 3
        self.d[1] = False
        
    def setDoneStatus(self):
        self.d[5] = False
    
    def isReady(self):
        return self.d[0]>0
        
    def isRunning(self):
        return self.d[0]==3
        
    def isPaused(self):
        return self.d[0]==5
        
    def getMeas(self, arg):
        self.d[4] = arg
        while not self.d[3]:
            pass
        self.d[3] = False
        return self.d[2]

    def sendSMS(self, text):
        """
        message = self.sms_client.messages \
                      .create(
                            body=text,
                            from_=self.sms_from, # this is my twilio number
                            to=self.sms_to # this is my number
                      )
        """
        print('message sent!')

payload = genericPayload()

if __name__ == '__main__':
    payload.ser.write('$S0&')
    
