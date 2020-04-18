#!/usr/bin/env python

# This is the main script. The UI people can create this for us.
# It calls to main_control which reads in sensor data and controls the system.
import time
import sensors
import alarms
import controls
import helper
import numpy as np
from os import remove

# Initialize sensor reading/tracking and UI structures:
jp               = sensors.JuliePlease()                # Provides a higher-level sensor interface.
alarm_bounds     = alarms.AlarmBounds()                 # Intiialize object for storing alarm trigger bounds.
controller       = controls.Controller(jp)

def take_step(control,alarm_bounds):
 
  # Control loop. Update controller according to tracked sensor values. 
  controller.update()

  #Throw any alarms that need throwing.
  #alarms.throw_raw_alarms(controller.logger, alarm_bounds)

try:
  while True:
    # UI logic should go into this script. Callbacks needed. 
    take_step(controller, alarm_bounds)

    # Pause until next datapoint capture
    time.sleep(controller.settings.SAMPLETIME)

except KeyboardInterrupt:
  print("Ctl C pressed - ending program")
  #remove('log.csv')
  #np.savetxt('log.csv',controller.logger.data,fmt='%5.2f',delimiter=',',header='time,state,p1,p2,o2,flow,temp,humd,btime,t_vol')
  del controller
