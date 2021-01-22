#!/usr/bin/env python
from geometry_msgs.msg import PoseWithCovarianceStamped
import os, time

def gotoWaitAndReturn(task, payload, waypointFileName, sleepTime=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=0')
    payload.pause()
    if s==0:
        if sleepTime>0:
            time.sleep(sleepTime)
            payload.resume()
    while payload.isPaused():
        pass
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=1 _isBackup:=1')
    return s

def gotoWaitAndProceed(task, payload, waypointFileName, sleepTime=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:=0')
    payload.pause()
    if s==0:
        if sleepTime>0:
            time.sleep(sleepTime)
            payload.resume()
    while payload.isPaused():
        pass
    task.waypt_idx += 1
    return s
    
def gotoAndProceed(task, payload, waypointFileName, direction=0):
    if hasattr(task, 'isMirror'):
        isMirror = task.isMirror
    else:
        isMirror = 0
    s = os.system('rosrun pcv_base gotoPos.py _location:='+waypointFileName+' _direction:='\
                  +str(direction)+' _isMirror:='+str(isMirror)+' _isBackup:=0')
    task.waypt_idx += 1
    return s

def markerFreeVisualServoingWaitAndReturn(task, payload, approachTraj, targetImage, desX=1.8, desY=-0.1, desTh=0., sleepTime=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=0')
    s = os.system('rosrun pcv_base marker_free_visual_servo.py _targetName:='+targetImage+\
                  ' _desX:='+str(desX)+' _desY:='+str(desY)+' _desTh:='+str(desTh))
    payload.pause()
    if s==0:
        if sleepTime>0:
            time.sleep(sleepTime)
            payload.resume()
    while payload.isPaused():
        pass
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=1 _isBackup:=1')
    task.waypt_idx += 1
    return s

def markerBasedVisualServoingWaitAndReturn(task, payload, approachTraj, idL, idR, desX=1.5, desY=0., desTh=0., sleepTime=-1):
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=0')
    s = os.system('rosrun pcv_base aruco_visual_servo.py _refIDLeft:='+str(idL)+' _refIDRight:='+str(idR)+\
                  ' _desX:='+str(desX)+' _desY:='+str(desY)+' _desTh:='+str(desTh))
    payload.pause()
    if s==0:
        if sleepTime>0:
            time.sleep(sleepTime)
            payload.resume()
    while payload.isPaused():
        pass
    s = os.system('rosrun pcv_base gotoPos.py _location:='+approachTraj+' _direction:=1 _isBackup:=1')
    task.waypt_idx += 1
    return s
