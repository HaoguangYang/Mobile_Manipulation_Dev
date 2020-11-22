#!/usr/bin/env python

import serial
import time
import multiprocessing
from twilio.rest import Client
import xml.etree.ElementTree as ET 

class disinfectionPayload():
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
        self.d[1] = 0.0     # current (A)
        self.d[2] = False   # lampCmd
        self.d[3] = True    # rst_nCmd (Software-generated)
        #try:
        #    thread.start_new_thread( self.readSerial )
        #except:
        #    print "Error: unable to start Serial Listener thread"

    def serialComm(self):
        self.ser.write('$S0&$L0&')
        lampState = False
        sms_timeout = 3600
        sms_time = 0
        lowCurrentFlag = 0
        while 1:
            if self.ser.inWaiting():
                header = self.ser.read(1)            # read one byte
                if header == '@':               # header captured
                    cmd = self.ser.read(1)           # read second byte     
                    if cmd == 'A':
                        current = self.ser.read(4)
                        #print(current)
                        end = self.ser.read(1)
                        if end == '%':          # tail matches, valid msg.
                            self.d[1] = (float(current)-512.0)/4.83 # compose ros message of lamp current
                            if lampState and self.d[1] < 12.0:
                                #print self.d[1]
                                # delay, send sms until 20 consecutive low measurements.
                                #print(lowCurrentFlag)
                                lowCurrentFlag = lowCurrentFlag + 1
                            elif lampState and self.d[1] >= 12.0:
                                lowCurrentFlag = 0
                    elif cmd == 'S':
                        status = self.ser.read(1)
                        end = self.ser.read(1)
                        if end == '%':          # tail matches, valid msg
                            if status == '1':
                                self.d[0] = True# start the code
                            else:
                                self.d[0] = False
                    else:
                        # kill everything but keyboard teleop.
                        pass
                else:                           # not a message.
                    pass
            if self.d[2] == True and lampState == False:
                self.ser.write('$L1&')
                lampState = True
            elif self.d[2] == False and lampState == True:
                self.ser.write('$L0&')
                lampState = False
            if self.d[3] == False:
                if self.d[0] == True:
                    self.ser.write('$L0&')
                    lampState = False
                    self.ser.write('$S0&')
                self.d[3] = True
                self.d[0] = False
            if lowCurrentFlag > 20:
                timeNow = time.time()
                if timeNow - sms_time > sms_timeout :
                    # send text msg
                    message = self.sms_client.messages \
                              .create(
                                    body='FIXME: UVC Lamp Battery Low or Lamp is Broken!',
                                    from_=self.sms_from, # this is my twilio number
                                    to=self.sms_to # this is my number
                              )
                    print('message sent!')
                    sms_time = timeNow
            #print(self.d)

    def initialize(self):
        self.ser.write('$S0&$L0&')
        self.payload_thread = multiprocessing.Process(target=self.serialComm, args=())
        self.payload_thread.daemon = True
        self.payload_thread.start()
        
    def turnOnUVC(self):
        self.d[2] = True
    
    def turnOffUVC(self):
        self.d[2] = False
        
    def setDoneStatus(self):
        self.d[3] = False
    
    def isReady(self):
        return self.d[0]
        
    def isOn(self):
        return self.d[2]
        
    def getCurrent(self):
        return self.d[1]

payload = disinfectionPayload()

if __name__ == '__main__':
    payload.ser.write('$S0&$L0&')
    
