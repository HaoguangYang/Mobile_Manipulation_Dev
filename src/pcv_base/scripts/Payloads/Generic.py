#!/usr/bin/env python

import serial
import time
import multiprocessing
from twilio.rest import Client
import xml.etree.ElementTree as ET 

class genericPayload():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', timeout=2)  # open serial port.
        time.sleep(1)
        self.ser.setDTR(0)
        time.sleep(1)
        sms_cred = ET.parse('/home/cartman/Dev/smsCredentials.xml')
        account_sid = sms_cred.findall('account_sid')[0].get('value')
        auth_token = sms_cred.findall('auth_token')[0].get('value')
        self.sms_client = Client(account_sid, auth_token)
        self.sms_from = sms_cred.findall('from')[0].get('value')
        self.sms_to = sms_cred.findall('to')[0].get('value')
        self.mgr = multiprocessing.Manager()
        self.d = self.mgr.dict()
        self.d[0] = False   # isReady
        self.d[1] = False   # isPaused
        self.d[2] = -1      # measurement to query
        self.d[3] = 0       # some analog value
        self.d[4] = False   # new meas value available
        self.d[5] = True    # rst_nCmd (Software-generated)
        #try:
        #    thread.start_new_thread( self.readSerial )
        #except:
        #    print "Error: unable to start Serial Listener thread"

    def serialComm(self):
        self.ser.write('$S0&')
        while 1:
            if self.ser.inWaiting():
                header = self.ser.read(1)            # read one byte
                if header == '@':               # header captured
                    cmd = self.ser.read(1)           # read second byte     
                    if cmd == 'A':
                        meas = self.ser.read(4)
                        end = self.ser.read(1)
                        if end == '%':          # tail matches, valid msg.
                            self.d[3] = int(meas) # save current analog measurement
                            self.d[4] = True
                    elif cmd == 'S':
                        status = self.ser.read(1)
                        end = self.ser.read(1)
                        if end == '%':          # tail matches, valid msg
                            if status == '0':
                                self.d[0] = False
                            else:
                                if status = '3':
                                    self.d[0] = True    # start the code, out of pause or go to next waypoint
                                    self.d[1] = False
                    else:
                        pass
                else:                           # not a message.
                    pass
            if self.d[2] >= 0:
                self.ser.write('$Q'+str(self.d[2])+'&')
                self.d[2] = -1
            if self.d[1]:
                self.ser.write('$S5&')
            if self.d[5] == False:
                if self.d[0] == True:
                    self.ser.write('$S0&')
                self.d[5] = True
                self.d[0] = False
                self.d[1] = False

    def initialize(self):
        self.ser.write('$S0&')
        self.payload_thread = multiprocessing.Process(target=self.serialComm, args=())
        self.payload_thread.daemon = True
        self.payload_thread.start()
        
    def pause(self):
        self.d[1] = True
    
    def resume(self):
        self.d[1] = False
        
    def setDoneStatus(self):
        self.d[5] = False
    
    def isReady(self):
        return self.d[0]
        
    def isRunning(self):
        return self.d[1]
        
    def getMeas(self, arg):
        self.d[2] = arg
        while not self.d[4]:
            pass
        self.d[4] = False
        return self.d[3]

    def sendSMS(self, text):
        message = self.sms_client.messages \
                      .create(
                            body=text,
                            from_=self.sms_from, # this is my twilio number
                            to=self.sms_to # this is my number
                      )
        print('message sent!')

payload = genericPayload()

if __name__ == '__main__':
    payload.ser.write('$S0&')
    
