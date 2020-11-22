#!/usr/bin/env python
#import requests
#from payload import payload  #Used to get UVC lamp data
import pymysql.cursors
from datetime import datetime
import time
from pcv_base.msg import electricalStatus
from nav_msgs.msg import Odometry
import rospy
from datetime import datetime
import xml.etree.ElementTree as ET 

class SQL_Logger:
    def __init__(self):
        db_cred = ET.parse('/home/cartman/Dev/dbCredentials.xml')
        server = db_cred.findall('server')[0].get('value')
        usrname = db_cred.findall('user')[0].get('value')
        passwd = db_cred.findall('password')[0].get('value')
        dbName = db_cred.findall('databaseName')[0].get('value')
        self.connection = pymysql.connect(host=server, user=usrname, password=passwd, db=dbName)    # Fill in your credentials  
        self.staleValues = True
        self.steer_1_Volt = 0.
        self.steer_2_Volt = 0.
        self.steer_3_Volt = 0.
        self.steer_4_Volt = 0.

        self.roll_1_Volt = 0.
        self.roll_2_Volt = 0.
        self.roll_3_Volt = 0.
        self.roll_4_Volt = 0.
        
        self.steer_1_Amp = 0.
        self.steer_2_Amp = 0.
        self.steer_3_Amp = 0.
        self.steer_4_Amp = 0.

        self.roll_1_Amp = 0.
        self.roll_2_Amp = 0.
        self.roll_3_Amp = 0.
        self.roll_4_Amp = 0.
        
        self.orientation = 0.
        self.angVel = 0.
        self.locX = 0.
        self.locY = 0.
        self.linVelX = 0.
        self.linVelY = 0.
        
        self.payloadCurrent = 0.
        self.payloadState = False

        self.date = datetime.now()
    
    def callbackElec(self,d):
        #timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.date = datetime.fromtimestamp(d.stamp.to_sec())

        self.steer_1_Volt = d.steer_1_Volt
        self.steer_2_Volt = d.steer_2_Volt
        self.steer_3_Volt = d.steer_3_Volt
        self.steer_4_Volt = d.steer_4_Volt

        self.roll_1_Volt = d.roll_1_Volt
        self.roll_2_Volt = d.roll_2_Volt
        self.roll_3_Volt = d.roll_3_Volt
        self.roll_4_Volt = d.roll_4_Volt
        
        self.steer_1_Amp = d.steer_1_Amp
        self.steer_2_Amp = d.steer_2_Amp
        self.steer_3_Amp = d.steer_3_Amp
        self.steer_4_Amp = d.steer_4_Amp

        self.roll_1_Amp = d.roll_1_Amp
        self.roll_2_Amp = d.roll_2_Amp
        self.roll_3_Amp = d.roll_3_Amp
        self.roll_4_Amp = d.roll_4_Amp
        
        self.staleValues = False
        
    def callbackOdom(self,d):
        #self.orientation = d.pose.pose.orientation.z #Need to covnert to Euler Angle
        self.angVel = d.twist.twist.angular.z

        self.locX = d.pose.pose.position.x
        self.locY = d.pose.pose.position.y
        self.linVelX = d.twist.twist.linear.x
        self.linVelY = d.twist.twist.linear.y

    def uploadData(self):
        # Add data to DB
        try:
            with self.connection.cursor() as cursor:
                
                # INSERT INTO [TABLE NAME] (COLUMN NAME) VALUE(value1, value2)...

                #Casts all parameters to strings, database can still intpret it
                sql = "INSERT INTO `Omniveyors`.`CARTMAN12_practice` \
                        (`TimeStamp`, `steer1Volt`, `roll1Volt`, `steer2Volt`, \
                        `roll2Volt`, `steer3Volt`, `roll3Volt`, `steer4Volt`, \
                        `roll4Volt`, `steer1Amp`, `roll1Amp`, `steer2Amp`, \
                        `roll2Amp`, `steer3Amp`, `roll3Amp`, `steer4Amp`, \
                        `roll4Amp`, `Orientation`,`AngVel`, `Location`, `LinVel`,\
                        `PayloadCurrent`, `PayloadState`\
                        ) VALUES(\
                        '"+str(self.date)+"','"+str(self.steer_1_Volt)+"',\
                        '"+str(self.roll_1_Volt)+"','"+str(self.steer_2_Volt)+"',\
                        '"+str(self.roll_2_Volt)+"','"+str(self.steer_3_Volt)+"',\
                        '"+str(self.roll_3_Volt)+"','"+str(self.steer_4_Volt)+"',\
                        '"+str(self.roll_4_Volt)+"','"+str(self.steer_1_Amp)+"',\
                        '"+str(self.roll_1_Amp)+"','"+str(self.steer_2_Amp)+"',\
                        '"+str(self.roll_2_Amp)+"','"+str(self.steer_3_Amp)+"',\
                        '"+str(self.roll_3_Amp)+"','"+str(self.steer_4_Amp)+"',\
                        '"+str(self.roll_4_Amp)+"','"+str(self.orientation)+"',\
                        '"+str(self.angVel)+"',\
                        POINT('"+str(self.locX)+"','"+str(self.locY)+"'), \
                        POINT('"+str(self.linVelX)+"','"+str(self.linVelY)+"'),\
                        '"+str(self.payloadCurrent)+"','"+str(self.payloadState)+"');"
                #sql = "INSERT INTO `Omniveyors`.`CARTMAN12_practice` (`TimeStamp`, `steer1Volt`, `roll2Volt`, `steer2Volt`, `roll1Volt`, `steer3Volt`, `roll3Volt`, `steer4Volt`, `roll4Volt`) VALUES(%s %f %f %f %f %f %f %f %f);"
                #vals = (str(u.date),u.steer_1_Volt, u.roll_1_Volt, u.steer_2_Volt, u.roll_2_Volt, u.steer_3_Volt, u.roll_3_Volt, u.steer_4_Volt, u.roll_4_Volt) 

                #cursor.execute(sql, vals)
                cursor.execute(sql)
                result = cursor.fetchone()
                #print(sql)
        finally:
            #print("db..")
            self.connection.commit()
    
    def run(self):
        rospy.init_node("database_logger")
        rospy.Subscriber("electricalStatus", electricalStatus, self.callbackElec)
        rospy.Subscriber("odom", Odometry, self.callbackOdom)
        try:
            while not rospy.is_shutdown():
                time.sleep(10)
                if not self.staleValues:
                    #self.payloadCurrent = payload.getCurrent()
                    #self.payloadState = payload.isOn()
                    self.uploadData()
                    self.staleValues = True
        except:
            self.connection.close()
           

if __name__ == "__main__":
    dbLogger = SQL_Logger()
    dbLogger.run()
    
