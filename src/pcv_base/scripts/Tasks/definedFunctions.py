#!/usr/bin/env python
from geometry_msgs.msg import PoseWithCovarianceStamped
import os, time

def gotoWaitAndReturn(task, payload, waypointFileName, time=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=0')
    last_loc_rec = task.last_loc
    payload.pause()
    if s==0:
        if time>0:
            time.sleep(time)
            payload.resume()
    while payload.isPaused():
        pass
    task.pose_pub.publish(last_loc_rec)
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=1 _isBackup:=1')
    return s

def gotoWaitAndProceed(task, payload, waypointFileName, time=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=0')
    last_loc_rec = task.last_loc
    payload.pause()
    if s==0:
        if time>0:
            time.sleep(time)
            payload.resume()
    while payload.isPaused():
        pass
    task.pose_pub.publish(last_loc_rec)
    task.waypt_idx += 1
    return s
    
def gotoAndProceed(task, payload, waypointFileName, direction=0):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:='\
                  +str(direction)+' _isMirror:='+str(task.isMirror)+' _isBackup:=0')
    task.waypt_idx += 1
    return s

def markerFreeVisualServoingWaitAndReturn(task, payload, approachTraj, targetImage, desX=1.8, desY=-0.1, desTh=0., time=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=0')
    s = os.system('rosrun pcv_base marker_free_visual_servo.py _targetName:='+targetImage+\
                  ' _desX:='+str(desX)+' _desY:='+str(desY)+' _desTh:='+str(desTh))
    last_loc_rec = task.last_loc
    payload.pause()
    if s==0:
        if time>0:
            time.sleep(time)
            payload.resume()
    while payload.isPaused():
        pass
    task.pose_pub.publish(last_loc_rec)
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=1 _isBackup:=1')
    return s

def markerBasedVisualServoingWaitAndReturn(task, payload, approachTraj, idL, idR, desX=1.5, desY=0., desTh=0., time=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=0')
    s = os.system('rosrun pcv_base aruco_visual_servo.py _refIDLeft:='+str(idL)+' _refIDRight:='+str(idR)+\
                  ' _desX:='+str(desX)+' _desY:='+str(desY)+' _desTh:='+str(desTh))
    last_loc_rec = task.last_loc
    payload.pause()
    if s==0:
        if time>0:
            time.sleep(time)
            payload.resume()
    while payload.isPaused():
        pass
    task.pose_pub.publish(last_loc_rec)
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=1 _isBackup:=1')
    return s
