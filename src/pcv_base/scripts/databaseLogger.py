#!/usr/bin/env python
#import requests
#from payload import payload  #Used to get UVC lamp data
import pymysql.cursors
from datetime import datetime
import time
import socket
from tf import transformations as ts
from pcv_base.msg import electricalStatus
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
        usrname = db_cred.findall('user')[0].get('value')
        passwd = db_cred.findall('password')[0].get('value')
        dbName = db_cred.findall('databaseName')[0].get('value')
        self.tableName = db_cred.findall('tableName')[0].get('value')
        self.connection = pymysql.connect(host=server, user=usrname, password=passwd, db=dbName)    # Fill in your credentials  
        self.UTC_OFFSET_TIMEDELTA = datetime.utcnow() - datetime.now()

        self.robot_ip = None
        self.newNavStat = False
        
        self.counter = 0
        self.upload_cyles = 0
        
        self.steer_1_Amp = 0.
        self.steer_2_Amp = 0.
        self.steer_3_Amp = 0.
        self.steer_4_Amp = 0.
        self.roll_1_Amp = 0.
        self.roll_2_Amp = 0.
        self.roll_3_Amp = 0.
        self.roll_4_Amp = 0.
        
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
        self.linSpeed = 0.
        
        self.desX = 0.
        self.desY = 0.
        self.desOrient = 0.
        self.navState = 0
        
        self.payloadCurrent = 0.
        self.payloadState = 0

        self.date = datetime.now()
        
        # IP address will not change in a short period as long as it is connected,
        # so just record the startup IP.
        hostname = socket.gethostname()
        self.robot_ip = socket.gethostbyname(hostname)
        try:
            with self.connection.cursor() as cursor:
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...
                #Casts all parameters to strings, database can still interpret it
                sql = "INSERT INTO `Omniveyors`.`" + self.tableName + "` \
                        (`TimeStamp`, `IP` \
                        ) VALUES(\
                        '"+str(self.date)+"','"+self.robot_ip+"');"
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
        except:
            pass
    
    def callbackElec(self,d):
        #timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.date = datetime.fromtimestamp(d.stamp.to_sec()) + self.UTC_OFFSET_TIMEDELTA

        self.counter = self.counter + 1
        #self.upload_cyles = self.upload_cyles + 1
        
        self.battVoltSum = self.battVoltSum + \
                (d.steer_1_Volt + d.steer_2_Volt + d.steer_3_Volt + d.steer_4_Volt \
                + d.roll_1_Volt + d.roll_2_Volt + d.roll_3_Volt + d.roll_4_Volt)/8.0
        sumAmp = d.steer_1_Amp + d.steer_2_Amp + d.steer_3_Amp + d.steer_4_Amp \
                + d.roll_1_Amp + d.roll_2_Amp + d.roll_3_Amp + d.roll_4_Amp
        self.battAMax = max(self.battAMax, sumAmp)
        self.battAmpSum = self.battAmpSum + sumAmp
        
        self.steer_1_Amp = self.steer_1_Amp + d.steer_1_Amp
        self.steer_2_Amp = self.steer_2_Amp + d.steer_2_Amp
        self.steer_3_Amp = self.steer_3_Amp + d.steer_3_Amp
        self.steer_4_Amp = self.steer_4_Amp + d.steer_4_Amp

        self.roll_1_Amp = self.roll_1_Amp + d.roll_1_Amp
        self.roll_2_Amp = self.roll_2_Amp + d.roll_2_Amp
        self.roll_3_Amp = self.roll_3_Amp + d.roll_3_Amp
        self.roll_4_Amp = self.roll_4_Amp + d.roll_4_Amp
        
        self.s1AMax = min(max(d.steer_1_Amp, abs(self.s1AMax)),100.)
        self.s2AMax = min(max(d.steer_2_Amp, abs(self.s2AMax)),100.)
        self.s3AMax = min(max(d.steer_3_Amp, abs(self.s3AMax)),100.)
        self.s4AMax = min(max(d.steer_4_Amp, abs(self.s4AMax)),100.)
        self.r1AMax = min(max(d.roll_1_Amp, abs(self.r1AMax)),100.)
        self.r2AMax = min(max(d.roll_2_Amp, abs(self.r2AMax)),100.)
        self.r3AMax = min(max(d.roll_3_Amp, abs(self.r3AMax)),100.)
        self.r4AMax = min(max(d.roll_4_Amp, abs(self.r4AMax)),100.)
        
    def callbackAMCL(self,d):
        self.locX = d.pose.pose.position.x
        self.locY = d.pose.pose.position.y
        quat = [d.pose.pose.orientation.x, d.pose.pose.orientation.y, \
                d.pose.pose.orientation.z, d.pose.pose.orientation.w]
        eul = ts.euler_from_quaternion(quat)
        self.orientation = eul[2]
        
    def callbackOdom(self,d):
        self.linSpeed = np.sqrt(d.twist.twist.linear.x**2 + d.twist.twist.linear.y**2)
        self.angVel = d.twist.twist.angular.z

    def status_cb(self,d):
        # nav status will not publish until the status changes.
        self.navState = d.status.status
        self.uploadNavData()

    def uploadNavData(self): #Includes the destination location, orientation, and the robot's IP
        # Add data to DB
        try:
            with self.connection.cursor() as cursor:
                
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...

                #Casts all parameters to strings, database can still interpret it
                sql = "INSERT INTO `Omniveyors`.`" + self.tableName + "` \
                        (`TimeStamp`, `PosX`, `PosY`, `Orientation`, `DesX`, `DesY`, `DesOrient`, \
                        `LinSpeed`, `AngVel`, `NavStatus`, \
                        `PayloadCurrent`, `PayloadState` \
                        ) VALUES(\
                        '"+str(self.date)+"', \
                        '"+str(round(self.locX,4))+"','"+str(round(self.locY,4))+"','"+str(round(self.orientation,4))+"',\
                        '"+str(round(self.desX,4))+"','"+str(round(self.desY,4))+"','"+str(round(self.desOrient,4))+"',\
                        '"+str(round(self.linSpeed,3))+"','"+str(round(self.angVel,3))+"','"+str(self.navState)+"',\
                        '"+str(self.payloadCurrent)+"','"+str(self.payloadState)+"');"
                
                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
        finally:
            #print("db..ip")
            self.connection.commit()
            
    def uploadData(self):
        # Add data to DB
        try:
            with self.connection.cursor() as cursor:
                
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...

                #Casts all parameters to strings, database can still interpret it
                sql = "INSERT INTO `Omniveyors`.`" + self.tableName + "` \
                        (`TimeStamp`, `BattVolt`, `steer1Amp`, `roll1Amp`, `steer2Amp`, \
                        `roll2Amp`, `steer3Amp`, `roll3Amp`, `steer4Amp`, `roll4Amp`, \
                        `steer1AMax`, `roll1AMax`, `steer2AMax`, `roll2AMax`, \
                        `steer3AMax`, `roll3AMax`, `steer4AMax`, `roll4AMax`, \
                        `BattAmp`, `BattAMax`, \
                        `PosX`, `PosY`, `Orientation`, \
                        `LinSpeed`, `AngVel`, \
                        `PayloadCurrent`, `PayloadState`\
                        ) VALUES(\
                        '"+str(self.date)+"','"+str(round(self.battVoltSum/float(self.counter),3))+"',\
                        '"+str(round(self.steer_1_Amp/float(self.counter),3))+"','"+str(round(self.roll_1_Amp/float(self.counter),3))+"', \
                        '"+str(round(self.steer_2_Amp/float(self.counter),3))+"','"+str(round(self.roll_2_Amp/float(self.counter),3))+"', \
                        '"+str(round(self.steer_3_Amp/float(self.counter),3))+"','"+str(round(self.roll_3_Amp/float(self.counter),3))+"', \
                        '"+str(round(self.steer_4_Amp/float(self.counter),3))+"','"+str(round(self.roll_4_Amp/float(self.counter),3))+"', \
                        '"+str(round(self.s1AMax,3))+"','"+str(round(self.r1AMax,3))+"', \
                        '"+str(round(self.s2AMax,3))+"','"+str(round(self.r2AMax,3))+"', \
                        '"+str(round(self.s3AMax,3))+"','"+str(round(self.r3AMax,3))+"', \
                        '"+str(round(self.s4AMax,3))+"','"+str(round(self.r4AMax,3))+"', \
                        '"+str(round(self.battAmpSum/float(self.counter),3))+"', \
                        '"+str(round(self.battAMax,3))+"', \
                        '"+str(round(self.locX,4))+"','"+str(round(self.locY,4))+"','"+str(round(self.orientation,4))+"',\
                        '"+str(round(self.linSpeed,3))+"','"+str(round(self.angVel,3))+"',\
                        '"+str(self.payloadCurrent)+"','"+str(self.payloadState)+"');"
                
                #cursor.execute(sql, vals)
                self.upload_cyles = self.upload_cyles + 1
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
        finally:
            #print("db..normal")
            self.connection.commit()

    def run(self):
        rospy.init_node("database_logger")
        rospy.Subscriber("electricalStatus", electricalStatus, self.callbackElec)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.callbackAMCL)
        rospy.Subscriber("odom", Odometry, self.callbackOdom)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_cb) #Need to test if this can access the navigation status
        
        while not rospy.is_shutdown():
            time.sleep(10)
            if self.counter: #Checks if the counter is more than 0
                #self.payloadCurrent = payload.getCurrent()
                #self.payloadState = payload.isOn()
                self.uploadData()
                self.counter = 0
                self.battVoltSum = 0.
                self.batAmpSum = 0.
                self.battAMax = 0.
                self.steer_1_Amp = 0.
                self.steer_2_Amp = 0.
                self.steer_3_Amp = 0.
                self.steer_4_Amp = 0.
                self.roll_1_Amp = 0.
                self.roll_2_Amp = 0.
                self.roll_3_Amp = 0.
                self.roll_4_Amp = 0.
                self.s1AMax = 0.
                self.s2AMax = 0.
                self.s3AMax = 0.
                self.s4AMax = 0.
                self.r1AMax = 0.
                self.r2AMax = 0.
                self.r3AMax = 0.
                self.r4AMax = 0.
        try:
            pass
        finally:
            self.connection.close()
           

if __name__ == "__main__":
    dbLogger = SQL_Logger()
    dbLogger.run()
    

