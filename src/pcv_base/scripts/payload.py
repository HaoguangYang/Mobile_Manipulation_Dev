import serial
import time
import multiprocessing
from functools import wraps

ser = serial.Serial('/dev/ttyACM0', timeout=2)  # open serial port.

class disinfectionPayload():
    def __init__(self):
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

    def serialComm(self, d):
        ser.write('$L0&')
        lampState = False
        sms_timeout = 3600
        sms_time = 0
        while 1:
            if ser.inWaiting():
                header = ser.read(1)            # read one byte
                if header == '@':               # header captured
                    cmd = ser.read(1)           # read second byte     
                    if cmd == 'A':
                        current = ser.read(3)
                        end = ser.read(1)
                        if end == '%':          # tail matches, valid msg.
                            d[1] = (float(current)-512.0)/4.83 # compose ros message of lamp current
                    elif cmd == 'S':
                        status = ser.read(1)
                        end = ser.read(1)
                        if end == '%':          # tail matches, valid msg
                            if status == '1':
                                d[0] = True# start the code
                            else:
                                d[0] = False
                    else:
                        # kill everything but keyboard teleop.
                        pass
                else:                           # not a message.
                    pass
            if d[2] == True and lampState == False:
                ser.write('$L1&')
                lampState = True
            elif d[2] == False and lampState == True:
                ser.write('$L0&')
                lampState = False
            if d[3] == False and d[0] == True:
                ser.write('$S0%')
                d[3] = True
                d[0] = False
            if self.d[1] < 12.0:
                timeNow = time.time()
                if timeNow - sms_time > sms_timeout :
                    # send text msg
                    sms_time = timeNow

    def turnOnUVC(self):
        self.d[2] = True
    
    def turnOffUVC(self):
        self.d[2] = False
        
    def setDoneStatus(self):
        self.d[3] = False
    
    def isReady(self):
        return self.d[0]
        
    def getCurrent(self):
        return self.d[1]
        
payload = disinfectionPayload()
payload_thread = multiprocessing.Process(target=payload.serialComm, args=(payload.d,))
payload_thread.daemon = True
payload_thread.start()

if __name__ == '__main__':
    pass
