#!/usr/bin/env python
import pyserial
import rospy

def Payload(base):
    class PayloadClass(base):
        def __init__(self):
            super(PayloadClass, self).__init__()
            self.__payload_if = serial.Serial('/dev/ttyACM0', timeout=2)  # open serial port.
            time.sleep(1)
            self.__payload_if.setDTR(0)
            time.sleep(1)
            """
            sms_cred = ET.parse('/home/cartman/Dev/smsCredentials.xml')
            account_sid = sms_cred.findall('account_sid')[0].get('value')
            auth_token = sms_cred.findall('auth_token')[0].get('value')
            self.sms_client = Client(account_sid, auth_token)
            self.sms_from = sms_cred.findall('from')[0].get('value')
            self.sms_to = sms_cred.findall('to')[0].get('value')
            """
            self.payload_in_queue = ""
            self.payload_out_queue = ""
            
        def execute(self):
            super(PayloadClass, self).execute()
            self.__payload_if_cycle_inst = rospy.Timer(rospy.Duration(1./50.), self.__cycle)
        
        def __cycle(self):
            in_bytes = self.__payload_if.inWaiting()
            out_bytes = len(self.payload_out_queue)
            if in_bytes:
                self.payload_in_queue += self.__payload_if.read(in_bytes)
                # recycle space
                in_size = len(self.payload_in_queue)
                if in_size > 256:
                    self.payload_in_queue = self.payload_in_queue[in_size-256:]
            if out_bytes:
                self.__payload_if.write(self.payload_out_queue)
                self.payload_out_queue = ""
                
        def ser_recv(head, tail, size):
            offset_head = len(head)
            offset_tail = len(tail)
            # find beginning of the message
            start = self.payload_in_queue.find(head)
            # extract payload
            data = self.payload_in_queue[start+offset_head:start+offset_head+size]
            # find end of the message
            end = self.payload_in_queue.find(tail, start+offset_head+size)
            # remove message from queue
            self.payload_in_queue = self.payload_in_queue[0:start]+self.payload_in_queue[end+offset_tail:]
            if end!=start+offset_head+size:
                # Size of payload does not match the size provided. Invalidate message
                return ""
            else:
                # return the payload stripped from the in queue as a string.
                return data
        
        def ser_send(msg):
            self.payload_out_queue += msg
    return PayloadClass

if __name__ == '__main__':
    pass
