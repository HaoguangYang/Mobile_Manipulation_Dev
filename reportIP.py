#!/usr/bin/env python
import pymysql.cursors
import socket
from datetime import datetime
import time
import netifaces as ni
import os
import numpy as np
from datetime import datetime
import xml.etree.ElementTree as ET

class SQL_IP_Logger:
    def __init__(self):
        db_cred = ET.parse('/home/cartman/Dev/dbCredentials.xml')
        server = db_cred.findall('server')[0].get('value')
        portNo = db_cred.findall('port')
        if len(portNo)>0:
            portNo = int(portNo[0].get('value'))
        else:
            portNo = 3306
        usrname = db_cred.findall('user')[0].get('value')
        passwd = db_cred.findall('password')[0].get('value')
        self.dbName = db_cred.findall('databaseName')[0].get('value')
        thisNode = os.getenv('NODE_NO')
        print (thisNode)
        self.telemetryTableName = 'CARTMAN'+(thisNode.zfill(2))+'_TELEMETRY'    #db_cred.findall('telemetryTableName')[0].get('value')
        sock_test = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                not_up = sock_test.connect_ex((server, portNo))
                if not not_up:
                    break
            except:
                pass
        self.connection = pymysql.connect(host=server, port=portNo, user=usrname, password=passwd, database=self.dbName)    # Fill in your credentials  
        self.UTC_OFFSET_TIMEDELTA = datetime.utcnow() - datetime.now()

        self.robot_ip = None
        
        self.date = datetime.utcnow() #datetime.now()
        #print("Starting..")
        
        # IP address will not change in a short period as long as it is connected,
        # so just record the startup IP.
        ifaces = ni.interfaces()
        wlan_name = ''
        for item in ifaces:
            if 'wlx' in item:
                wlan_name = item
                break
        self.robot_ip = ni.ifaddresses(wlan_name)[ni.AF_INET][0]['addr']
        print (self.robot_ip)

    def reportIP(self):
        with self.connection.cursor() as cursor:
            # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
            sql = "INSERT INTO `"+self.dbName+"`.`" + self.telemetryTableName + "` \
                    (`TimeStamp`, `IP`) VALUES(\
                    '"+str(self.date)+"', '"+self.robot_ip+"');"
            #cursor.execute(sql, vals)
            cursor.execute(sql)
            result = cursor.fetchone()
            print(sql)
        #print("db..navigation")
        self.connection.commit()
        print('telemetry sent')
        self.connection.close()
           

if __name__ == "__main__":
    dbLogger = SQL_IP_Logger()
    dbLogger.reportIP()
    

