#!/usr/bin/env python
from Base import *
from Payload import *
from Task import *

def assembleTask(base, payload, task):
    base = eval(base)
    payload = eval(payload)
    task = eval(task)
    return task
    
