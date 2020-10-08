#!/usr/bin/env python
######################################################
# GUI control code for rally car
# This script creates the GUI thuogh which Rally car
#   can be semi-autonomously controlled
# The script uses parts of gui_example_students.py code
#   written by Dr. BC Min for creating the GUI
#
# Author         Version
#__________    ____________
# Mythra         25/4/18        mbalakun@purdue.edu
######################################################

# Dependencies
import argparse         # Parsing input port argument
import serial           # communicating with Roomba
import time             # For pause, sleep/delay
import Tkinter as tk    # For creating the GUI
from Tkinter import *
import tkMessageBox
from PIL import Image
import threading        # To creat threads
import numpy as np
from binascii import hexlify
import rospy
from std_msgs.msg import String
import math
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import os, signal
# custom imports
from msgsrv.srv import waypt
from msgsrv.msg import ctrlParams
import tf
from rospkg import RosPack

class rally_guictrl():
    def __init__(self, port):
    # Flags
        # Shutdown flag
        self.shtdwn = False      # To trigger shutdown procedures
        self.serflag = False     # To connect serial or not
        self.wayptflag = False   # To indicate if waypts are selected

    #Serial init parameters

        if self.serflag:
            self.ser = serial.Serial(port, 115200)
            self.ser.flush()
            self.ser.close()
            self.ser.open()
    # Gui stuff
        self.root = tk.Tk()
    # Control paramters
        self.cprms = ctrlParams()
        self.cprms.state = 1
        self.vel_lim = 0
        # [Kp_gas Kd_gas Ki_gas, Kp_steer Kd_steer Ki_steer]
        self.cprms.gains = [0.1, 0.1, 0.0, 0.1, 0.1, 0.0]

#######################################################
#################  Control function ###################
#######################################################
    def ctrl(self,state=-1):
        # Get the velocity set from GUI
        vel = self.vscale.get()
        # Clear canvas
        self.cnv.delete("all")

        if self.wayptflag or state == 2 or state == 3 or state == 6:
            # Stop
            if state == 0:
                self.cnv.create_oval(0, 0, 100, 100, width=0, fill='red')
                self.clog.insert(0.0,'Stop \n')
                # Send command to stop
                # ................ Needs to be written
                # Publish state and waypts
                self.cprms.state = 0
                self.cprms.vel_lim = 0
                self.cpub.publish(self.cprms)

            elif state == 1:                    # Start
                # Draw green circle to indicate start init
                self.cnv.create_oval(0, 0, 100, 100, width=0, fill='green')
                self.clog.insert(0.0,'Starting ... \n'+'-'*30+'\n')
                # Send command to start/continue robot motion
                # ................ Needs to be written - Process waypts
                # Invoke new terminal and run map follower with set waypts
                #gnome-terminal -e "rosrun"
                os.system("gnome-terminal -e 'rosrun wall_follower " + rospy.get_param('controller_file')+ "'")
                # pause(0.5)
                # Publish state and waypts
                self.cprms.state = 1
                self.cprms.vel_lim = (1.0*self.vscale.get())/100.0
                self.cpub.publish(self.cprms)
                self.clog.insert(0.0,'-'*30+'\n')
                self.clog.insert(0.0,'After init click Run to move car\n')

            elif state == 2:                    # Load waypts
                # Update display and text
                self.cnv.create_oval(0,0,100,100, width=0, fill='yellow')
                self.clog.insert(0.0, 'Selecting waypts..... \n')

                # Get selected waypts
                # Implement checks to ensure complete set is provided
                #  if not proceed until selected waypt
                # ................ Needs to be written
                # Load the waypts
                mrkr_tmp = Marker()
                for points in self.getwaypts(1).mrkr.poses:
                    print(points.pose.orientation)
                    ori = [points.pose.orientation.x, points.pose.orientation.y, points.pose.orientation.z, points.pose.orientation.w]
                    mrkr_tmp.points.append(Point(x=points.pose.position.x, y=points.pose.position.y,
                                               z=(tf.transformations.euler_from_quaternion(ori))[2]))
                self.waypts = mrkr_tmp.points
                #print('_'*50)
                #print(str(self.waypts[1].x))    # For debug
                #print(self.waypts)
                # Write waypts to the file
                wptfile = open(RosPack().get_path('wall_follower')+'/resources/ui_speedway.txt','w')
                # Write waypts to file
                for i in range(0,len(self.waypts)):
                    wptfile.write(str(self.waypts[i].x) + ' ' + str(self.waypts[i].y) + ' ' + str(self.waypts[i].z) + '\n')
                wptfile.close() #Close file

                self.wayptflag = True   # Set waypoints_loaded? flag to true
                self.clog.insert(0.0,'-'*30+'\n')
                self.clog.insert(0.0,'Set car at starting waypt then click Start\n')

            elif state == 3 and not self.wayptflag:                    # Auto
                # Update display and text
                self.cnv.create_oval(0,0,100,100, width=0, fill='yellow')
                self.clog.insert(0.0, 'Auto select waypts. \n')

                auto_wpts = np.loadtxt(RosPack().get_path('wall_follower')+'/resources/ui_auto_demo.txt')
                # Write waypts to the file
                wptfile = open(RosPack().get_path('wall_follower')+'/resources/ui_speedway.txt','w')
                # Write waypts to file
                for i in range(0,len(auto_wpts)):
                    wptfile.write(str(auto_wpts[i][0]) + ' ' + str(auto_wpts[i][1]) + ' ' + str(auto_wpts[i][2]) + '\n')
                wptfile.close() #Close file

                self.wayptflag = True   # Set waypoints_loaded? flag to true
                self.clog.insert(0.0,'-'*30+'\n')
                self.clog.insert(0.0,'Set car at starting waypt then click Start\n')

            # run
            elif state == 4:
                # Give options for Kp Kd tuning
                # Draw green directional triangle to indicate motion
                points = [50,0, 0,100, 100, 100]   # Set vertices
                self.cnv.create_polygon(points, fill='green', width=3)
                self.clog.insert(0.0,' Running..... \n')
                # Publish state and waypts
                self.cprms.vel_lim = (1.0*self.vscale.get())/100
                self.cprms.state = 1
                self.cpub.publish(self.cprms)

            # PD Tuning
            elif state == 5:
                # Give options for Kp Kd tuning
                # Draw directional triangle
                points = [50,0, 0,100, 100, 100]   # Set vertices
                self.cnv.create_polygon(points, fill='green', width=3)
                self.clog.insert(0.0,' Gains set and running...\n')
                # Publish gains, state and waypts
                self.cprms.vel_lim = (1.0*self.vscale.get())/100
                self.cprms.gains = [self.kpg.get(), self.kdg.get(), 0,\
                                    self.kps.get(), self.kds.get(), 0]
                self.cprms.vel_lim = (1.0*self.vscale.get())/100.0
                self.cprms.state = 1
                self.cpub.publish(self.cprms)
            # Del previous waypoint
            elif state == 6:
                # Call service with 0 to del prev waypt (Check waypt.srv in waypoint.py)
                self.getwaypts(0)
            # Reset controller
            elif state == 7:
                # Call service with -1 to reset (Check waypt.srv in waypoint.py)
                self.getwaypts(-1)
                self.clog.insert(0.0,'-'*30+'\n')
                self.clog.insert(0.0,'Ensure localization is correct before restart\n')
                rospy.set_param('runPD_q',0)

        else:
                self.clog.insert(0.0,' Please select waypts first \n')

#######################################################
#### function which creates GUI create and runs #######
#######################################################
    def gui(self):
        # Make Gui window, title and background
        print('Setting up GUI')
        self.root.title("Rrrrally Gui - @Mythra")
        self.root.config(background = "#FFFFFF")

        rightFrame = tk.Frame(self.root, width=400, height = 600)
        rightFrame.grid(row=0, column=1, padx=10, pady=2)

        self.cnv = tk.Canvas(rightFrame, width=100, height=100, bg='white')
        self.cnv.grid(row=0, column=0, padx=10, pady=2)

        btnFrame = tk.Frame(rightFrame, width=200, height = 200)
        btnFrame.grid(row=1, column=0, padx=10, pady=2)

        # Create slider for velocity values
        self.vscale = Scale(btnFrame, from_=0, to=100, orient = HORIZONTAL)
        self.vscale.set(30) # Default speed
        self.vscale.grid(row=2, column=0, padx=10, pady=10)
        # Set label for above slider
        vlbl = Label(btnFrame, text = 'Max Speed(0-100%)')
        vlbl.grid(row=3, column=0, padx=20, pady=1)

        self.clog = tk.Text(rightFrame, width = 50, height = 10, takefocus=0)
        self.clog.grid(row=2, column=0, padx=10, pady=2)

        moveForwardBtn = tk.Button(btnFrame, text="Start", command=lambda: self.ctrl(1))
        moveForwardBtn.grid(row=0, column=1, padx=10, pady=2)

        moveLeftBtn = tk.Button(btnFrame, text="Load Waypts", command=lambda: self.ctrl(2))
        moveLeftBtn.grid(row=1, column=0, padx=10, pady=2)

        moveRightBtn = tk.Button(btnFrame, text="Auto", command= lambda: self.ctrl(3))
        moveRightBtn.grid(row=1, column=2, padx=10, pady=2)

        moveBackwardBtn = tk.Button(btnFrame, text="Stop", command= lambda: self.ctrl(0))
        moveBackwardBtn.grid(row=2, column=1, padx=10, pady=2)

        moveBackwardBtn = tk.Button(btnFrame, text="Reset", command= lambda: self.ctrl(7))
        moveBackwardBtn.grid(row=2, column=2, padx=10, pady=2)

        RunBtn = tk.Button(btnFrame, text="Run", command= lambda: self.ctrl(4))
        RunBtn.grid(row=1, column=1, padx=10, pady=2)

    # PID control settings
        pidFrame = tk.Frame(rightFrame, width=200, height = 200)
        pidFrame.grid(row=1, column=2, padx=10, pady=2)

        # Waypoints reset button
        wpt_resetBtn = tk.Button(pidFrame, text="Del Waypt", command= lambda: self.ctrl(6))
        wpt_resetBtn.grid(row=0, column=1, padx=10, pady=2)

        # PD tune button
        pdBtn = tk.Button(pidFrame, text="Update Params", command= lambda: self.ctrl(5))
        pdBtn.grid(row=0, column=0, padx=10, pady=2)

        # Kp_gas slider
        self.kpg = Scale(pidFrame, from_=0.0, to=2.0, digits = 3, resolution = 0.05, orient = HORIZONTAL)
        self.kpg.set(0.1) # Default value
        self.kpg.grid(row=1, column=0, padx=10, pady=10)
        # Set label for above slider
        kpg_lbl = Label(pidFrame, text = 'Kp_gas(0-2)')
        kpg_lbl.grid(row=2, column=0, padx=10, pady=1)
        # Kd_gas slider
        self.kdg = Scale(pidFrame, from_=0.0, to=2.0, digits = 3, resolution = 0.05, orient = HORIZONTAL)
        self.kdg.set(0.1) # Default value
        self.kdg.grid(row=1, column=1, padx=10, pady=10)
        # Set label for above slider
        kdg_lbl = Label(pidFrame, text = 'Kd_gas(0-2)')
        kdg_lbl.grid(row=2, column=1, padx=10, pady=1)
        # Kp_steer slider
        self.kps = Scale(pidFrame, from_=0.00, to=2.00,
                        digits = 3, resolution = 0.05, orient = HORIZONTAL)
        self.kps.set(0.1) # Default value
        self.kps.grid(row=3, column=0, padx=10, pady=10)
        # Set label for above slider
        kps_lbl = Label(pidFrame, text = 'Kp_steer(0-2)')
        kps_lbl.grid(row=4, column=0, padx=10, pady=1)
        # Kd_steer slider
        self.kds = Scale(pidFrame, from_=0.0, to=2.0,
                        digits = 3, resolution = 0.05, orient = HORIZONTAL)
        self.kds.set(0.1) # Default value
        self.kds.grid(row=3, column=1, padx=10, pady=10)
        # Set label for above slider
        kds_lbl = Label(pidFrame, text = 'Kd_steer(0-2)')
        kds_lbl.grid(row=4, column=1, padx=10, pady=1)



        # Create init text and images
        self.cnv.create_oval(0,0,100,100,width=0,fill='red')

        self.clog.insert(4.0,'Select Waypts on Map to start (use \'g\' to select each pt\n')
        self.clog.insert(6.0,'Click load waypts after select \n')
        self.clog.insert(2.0,'----------OR----------\n')
        self.clog.insert(1.0,'Click Auto for preset waypoints \n')




###################################################################
# Termination function, closes ports and kills all threads
    def shutdown(self):
        if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
            print('Shutting down ... Thanks!')
            # Set shtdwn flags
            self.shtdwn = True
            # Kill all threads
            self.root.destroy()
            # Kill ros node
            rospy.signal_shutdown('Killing Everything')

# Main function
    def main(self):
    # initialize ROS
        rospy.init_node('rally_gui', anonymous = True)
        # Create publisher for publishing Set pd gains and state from UI
        self.cpub = rospy.Publisher('control_params', ctrlParams, queue_size = 10)
        # Wait for waypoints service so we can obtain waypts selected
        rospy.wait_for_service('get_waypt')
        self.getwaypts = rospy.ServiceProxy('get_waypt',waypt)
        # Create gui
        self.gui()
        # Create thread to read sensors
        #sensethrd = threading.Thread(None, self.readSensors)
        #sensethrd.start()
        # Check for window close
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        # Start GUI
        self.root.mainloop()

########################################################################
# The start .......
if __name__ == "__main__":
    print('____________ RALLY-CAR GUI CONTROL _____________')
    # Parse input arguments
    # To check if port was specified
    parser = argparse.ArgumentParser(
        description='''Script to control rally car using GUI.
            Please quit by closing the GUI window (x on GUI), this ensures the code executes proper termination hooks( closing ports, stop OI robot,  kill infinite loops, etc). \n Check optional parameters to run below ''')
    parser.add_argument('--port', default = '/dev/ttyUSB0',
            help = 'Port name (provide if it is NOT %(default)s,  Ex: python rally_ui.py --port \'/dev/ttyUSB1\' or python rally_ui.py --port \'COM1\'')
    parser.add_argument('name')
    parser.add_argument('log')
    args = parser.parse_args()
    print('Setting port to ' + args.port)
    # Create rally_car_ui object
    rcar = rally_guictrl(args.port)
    # Start the guicontrol
    rcar.main()
