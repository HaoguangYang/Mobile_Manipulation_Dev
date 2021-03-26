#!/usr/bin/env python
from Base import *
from Payload import *
from Task import *

def assembleTask(base, payload, task):
    robot = eval(base)
    # Payload and Task has precedence:
    # Items added later will have knowledge and control over earlier items.
    for i in range(len(payload)):
        robot = eval(payload+'(robot)')
    for i in range(len(task)):
        robot = eval(task+'(robot)')
    return robot
    
