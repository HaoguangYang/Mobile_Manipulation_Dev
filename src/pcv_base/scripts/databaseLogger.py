#!/usr/bin/env python
#import requests
#from payload import payload  #Used to get UVC lamp data
import pymysql.cursors
from datetime import datetime
import time
import netifaces as ni
from tf import transformations as ts
import os
from pcv_base.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import numpy as np
from datetime import datetime
import xml.etree.ElementTree as ET
from move_base_msgs.msg import MoveBaseActionResult 

class SQL_Logger:
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
        self.motorTableName = 'CARTMAN'+(thisNode.zfill(2))+'_MOTOR'    #db_cred.findall('motorTableName')[0].get('value')
        self.navTableName = 'CARTMAN'+(thisNode.zfill(2))+'_NAV'        #db_cred.findall('navTableName')[0].get('value')
        self.payloadTableName = 'CARTMAN'+(thisNode.zfill(2))+'_PAYLOAD'        #db_cred.findall('payloadTableName')[0].get('value')
        self.telemetryTableName = 'CARTMAN'+(thisNode.zfill(2))+'_TELEMETRY'    #db_cred.findall('telemetryTableName')[0].get('value')
        self.connection = pymysql.connect(host=server, port=portNo, user=usrname, password=passwd, database=self.dbName)    # Fill in your credentials  
        self.UTC_OFFSET_TIMEDELTA = datetime.utcnow() - datetime.now()

        self.robot_ip = None
        
        self.newNavStat = False
        self.newPayloadStat = False
        self.counter = 0
        self.bcounter = 0
        
        # update intervals in seconds
        self.intervalMotor = 5.0
        self.intervalNav = 1.0
        self.intervalPayload = 1.0
        self.intervalTelemetry = 60.0
        
        # time last updated each table
        self.tLastMotor = datetime(1970,1,1,0,0,0,0)    # initialize
        self.tLastNav = datetime(1970,1,1,0,0,0,0)
        self.tLastPayload = datetime(1970,1,1,0,0,0,0)
        self.tLastTelemetry = datetime(1970,1,1,0,0,0,0)
        
        self.steer_1_Amp_Sum = 0.
        self.steer_2_Amp_Sum = 0.
        self.steer_3_Amp_Sum = 0.
        self.steer_4_Amp_Sum = 0.
        self.roll_1_Amp_Sum = 0.
        self.roll_2_Amp_Sum = 0.
        self.roll_3_Amp_Sum = 0.
        self.roll_4_Amp_Sum = 0.
        
        self.s1AMax = 0.
        self.s2AMax = 0.
        self.s3AMax = 0.
        self.s4AMax = 0.
        self.r1AMax = 0.
        self.r2AMax = 0.
        self.r3AMax = 0.
        self.r4AMax = 0.
        
        self.battVoltSum = 0.
        self.battAmpSum = 0.
        self.battAMax = 0.
        
        self.orientation = 0.
        self.angVel = 0.
        self.locX = 0.
        self.locY = 0.
        self.velX = 0.
        self.velY = 0.
        
        self.desX = 0.
        self.desY = 0.
        self.desOrient = 0.
        self.navState = 0
        
        self.payloadStatus = []

        self.date = datetime.now()
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
    
    def electrical_cb(self,d):
        #timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        #print("Electric callback triggered")
        self.date = datetime.fromtimestamp(d.stamp.to_sec()) #+ self.UTC_OFFSET_TIMEDELTA

        self.counter += 1
        self.bcounter += 1
        self.battVoltSum = self.battVoltSum + \
                (d.steer_1_Volt + d.steer_2_Volt + d.steer_3_Volt + d.steer_4_Volt \
                + d.roll_1_Volt + d.roll_2_Volt + d.roll_3_Volt + d.roll_4_Volt)/8.0
        sumAmp = d.steer_1_Amp + d.steer_2_Amp + d.steer_3_Amp + d.steer_4_Amp \
                + d.roll_1_Amp + d.roll_2_Amp + d.roll_3_Amp + d.roll_4_Amp + 1.0   # 1.0 for an estimation of the computer itself.
        self.battAMax = max(self.battAMax, abs(sumAmp))
        self.battAmpSum = self.battAmpSum + sumAmp
        
        self.steer_1_Amp_Sum = self.steer_1_Amp_Sum + d.steer_1_Amp
        self.steer_2_Amp_Sum = self.steer_2_Amp_Sum + d.steer_2_Amp
        self.steer_3_Amp_Sum = self.steer_3_Amp_Sum + d.steer_3_Amp
        self.steer_4_Amp_Sum = self.steer_4_Amp_Sum + d.steer_4_Amp

        self.roll_1_Amp_Sum = self.roll_1_Amp_Sum + d.roll_1_Amp
        self.roll_2_Amp_Sum = self.roll_2_Amp_Sum + d.roll_2_Amp
        self.roll_3_Amp_Sum = self.roll_3_Amp_Sum + d.roll_3_Amp
        self.roll_4_Amp_Sum = self.roll_4_Amp_Sum + d.roll_4_Amp
        
        self.s1AMax = max(abs(d.steer_1_Amp), self.s1AMax)
        self.s2AMax = max(abs(d.steer_2_Amp), self.s2AMax)
        self.s3AMax = max(abs(d.steer_3_Amp), self.s3AMax)
        self.s4AMax = max(abs(d.steer_4_Amp), self.s4AMax)
        self.r1AMax = max(abs(d.roll_1_Amp), self.r1AMax)
        self.r2AMax = max(abs(d.roll_2_Amp), self.r2AMax)
        self.r3AMax = max(abs(d.roll_3_Amp), self.r3AMax)
        self.r4AMax = max(abs(d.roll_4_Amp), self.r4AMax)
        
    def localization_cb(self,d):
        self.locX = d.pose.pose.position.x
        self.locY = d.pose.pose.position.y
        quat = [d.pose.pose.orientation.x, d.pose.pose.orientation.y, \
                d.pose.pose.orientation.z, d.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat)
        self.orientation = eul[2]
        self.newNavStat = True
        
    def odom_cb(self,d):
        self.velX = d.twist.twist.linear.x
        self.velY = d.twist.twist.linear.y
        self.angVel = d.twist.twist.angular.z
        #self.newNavStat = True

    def navStatus_cb(self,d):
        # nav status will not publish until the status changes.
        self.navState = d.status.status
        self.newNavStat = True
        
    def payload_cb(self,d):
        while d.index >= len(self.payloadStatus):
            self.payloadStatus.append([d.index, d.state, d.sensor])
        self.payloadStatus[d.index] = [d.index, d.state, d.sensor]
        self.newPayloadStat = True

    def uploadNavData(self):
        try:
            with self.connection.cursor() as cursor:
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
                #Casts all parameters to strings
                sql = "INSERT INTO `"+self.dbName+"`.`" + self.navTableName + "` \
                        (`TimeStamp`, `PosX`, `PosY`, `Orientation`, \
                        `VelX`, `VelY`, `AngVel`, \
                        `DesX`, `DesY`, `DesOrient`, `NavStatus`\
                        ) VALUES(\
                        '"+str(self.date)+"', \
                        '"+str(round(self.locX,4))+"','"+str(round(self.locY,4))+"','"+str(round(self.orientation,4))+"',\
                        '"+str(round(self.velX,3))+"','"+str(round(self.velY,3))+"','"+str(round(self.angVel,3))+"',\
                        '"+str(round(self.desX,4))+"','"+str(round(self.desY,4))+"','"+str(round(self.desOrient,4))+"','"+str(self.navState)+"');"
                
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
                self.newNavStat = False
        except:
            pass
        finally:
            #print("db..navigation")
            self.connection.commit()
            #print('nav status sent')
            
    def uploadMotorData(self):
        try:
            with self.connection.cursor() as cursor:
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
                #Casts all parameters to strings
                sql = "INSERT INTO `"+self.dbName+"`.`" + self.motorTableName + "` \
                        (`TimeStamp`, `steer1Amp`, `roll1Amp`, `steer2Amp`, \
                        `roll2Amp`, `steer3Amp`, `roll3Amp`, `steer4Amp`, `roll4Amp`, \
                        `steer1AMax`, `roll1AMax`, `steer2AMax`, `roll2AMax`, \
                        `steer3AMax`, `roll3AMax`, `steer4AMax`, `roll4AMax`\
                        ) VALUES(\
                        '"+str(self.date)+"',\
                        '"+str(round(self.steer_1_Amp_Sum/float(self.counter),3))+"','"+str(round(self.roll_1_Amp_Sum/float(self.counter),3))+"', \
                        '"+str(round(self.steer_2_Amp_Sum/float(self.counter),3))+"','"+str(round(self.roll_2_Amp_Sum/float(self.counter),3))+"', \
                        '"+str(round(self.steer_3_Amp_Sum/float(self.counter),3))+"','"+str(round(self.roll_3_Amp_Sum/float(self.counter),3))+"', \
                        '"+str(round(self.steer_4_Amp_Sum/float(self.counter),3))+"','"+str(round(self.roll_4_Amp_Sum/float(self.counter),3))+"', \
                        '"+str(round(self.s1AMax,3))+"','"+str(round(self.r1AMax,3))+"', \
                        '"+str(round(self.s2AMax,3))+"','"+str(round(self.r2AMax,3))+"', \
                        '"+str(round(self.s3AMax,3))+"','"+str(round(self.r3AMax,3))+"', \
                        '"+str(round(self.s4AMax,3))+"','"+str(round(self.r4AMax,3))+"');"
                
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
                
                self.counter = 0
                self.steer_1_Amp_Sum = 0.
                self.steer_2_Amp_Sum = 0.
                self.steer_3_Amp_Sum = 0.
                self.steer_4_Amp_Sum = 0.
                self.roll_1_Amp_Sum = 0.
                self.roll_2_Amp_Sum = 0.
                self.roll_3_Amp_Sum = 0.
                self.roll_4_Amp_Sum = 0.
                self.s1AMax = 0.
                self.s2AMax = 0.
                self.s3AMax = 0.
                self.s4AMax = 0.
                self.r1AMax = 0.
                self.r2AMax = 0.
                self.r3AMax = 0.
                self.r4AMax = 0.
        except:
            pass
        finally:
            #print("db..battery")
            self.connection.commit()
            #print('motor status sent')

    def uploadPayloadData(self):
        try:
            with self.connection.cursor() as cursor:
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
                sql = "INSERT INTO `"+self.dbName+"`.`" + self.payloadTableName + "` \
                        (`TimeStamp`, `PayloadSensor`, `PayloadState`\
                        ) VALUES(\
                        '"+str(self.date)+"', \
                        '"+str(round(self.payloadStatus[0][2],3))+"','"+str(int(self.self.payloadStatus[0][1]))+"');"
                
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
                self.newPayloadStat = False
        except:
            pass
        finally:
            #print("db..navigation")
            self.connection.commit()
            #print('payload status sent')
            
    def uploadTelemetryData(self):
        try:
            with self.connection.cursor() as cursor:
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
                sql = "INSERT INTO `"+self.dbName+"`.`" + self.telemetryTableName + "` \
                        (`TimeStamp`, `BattVolt`, `BattAmp`, `BattAMax`, `IP`\
                        ) VALUES(\
                        '"+str(self.date)+"', \
                        '"+str(round(self.battVoltSum/float(self.bcounter),3))+"','"+str(round(self.battAmpSum/float(self.bcounter),3))+"',\
                        '"+str(round(self.battAMax,3))+"','"+self.robot_ip+"');"
                
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
                self.bcounter = 0
                self.battVoltSum = 0.
                self.battAmpSum = 0.
                self.battAMax = 0.
        except:
            pass
        finally:
            #print("db..navigation")
            self.connection.commit()
            #print('telemetry sent')

    def run(self):
        rospy.init_node("database_logger")
        rospy.Subscriber("electricalStatus", electricalStatus, self.electrical_cb)
        rospy.Subscriber("payloadStatus", payloadStatus, self.payload_cb)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.localization_cb)
        rospy.Subscriber("odom", Odometry, self.odom_cb)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.navStatus_cb)
        
        while not rospy.is_shutdown():
            self.date = datetime.now()
            if (self.date - self.tLastTelemetry).total_seconds() > self.intervalTelemetry \
                and self.bcounter > 0:
                self.uploadTelemetryData()
                self.tLastTelemetry = self.date
            if (self.date - self.tLastMotor).total_seconds() > self.intervalMotor \
                and self.counter > 0:
                self.uploadMotorData()
                self.tLastMotor = self.date
            if (self.date - self.tLastPayload).total_seconds() > self.intervalPayload \
                and self.newPayloadStat :
                self.uploadPayloadData()
                self.tLastPayload = self.date
            if (self.date - self.tLastNav).total_seconds() > self.intervalNav \
                and self.newNavStat :
                self.uploadNavData()
                self.tLastNav = self.date
        try:
            pass
        finally:
            self.connection.close()
           

if __name__ == "__main__":
    dbLogger = SQL_Logger()
    dbLogger.run()
    

