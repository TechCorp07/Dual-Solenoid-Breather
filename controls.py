# library imports:
import time
import gpiozero
from sensors import JuliePlease
import numpy as np
#import RPi.GPIO as GPIO   # Import the GPIO library.

class ControlSettings:
# A class for obtaining sensor readings at a single timepoint. 
    def __init__(self):
        self.pip = 40.0       # PIP (peak inspiratory pressure) value, in cm H20
        self.peep = 5.0       # PEEP (positive-end expiratory pressure) value, in cm H20
        self.insptime = 2.0   # Inspiratory time, in seconds
        self.breathrate = 12  # Breath rate, in beats per minute
        self.vt = 500         # Tidal volume, in mL
        self.SAMPLETIME = 0.001

    def reset(self):
        self._state = 0
        self.pip = 35.0       # PIP (peak inspiratory pressure) value, in cm H20
        self.peep = 5.0       # PEEP (positive-end expiratory pressure) value, in cm H20
        self.insptime = 1.5   # Inspiratory time, in seconds
        self.breathrate = 20  # Breath rate, in beats per minute
        self.vt = 500         # Tidal volume, in mL
        self.SAMPLETIME = 0.01

class Controller:
    # This is to be the brains of the operation
    def __init__(self,jp):
        # Initlize sensor interface, datalogger, & controller settings
        self.jp     = jp
        self.logger = DataLogger(self.jp)
        self.settings = ControlSettings()
        # Sets up the GPIO interface for valve control.
        self.inlet = gpiozero.DigitalOutputDevice(17, active_high=True, initial_value=True) #starts open
        self.inspir = gpiozero.PWMOutputDevice(22, active_high=True, initial_value=1, frequency = 20) #starts closed
        self.expir = gpiozero.DigitalOutputDevice(27, active_high=False, initial_value=False) #Starts open
        # Initialize control state (assume I.C. is inhale) 
        self._state = 0
        #self.__inhale()
        # Keep's track of how long each state lasts
        self.start_time = time.time()

    def gpio_cleanup(self):
        # Powers off pins and cleans up GPIO after run is terminated. 
        self.inlet.close()
        self.inspir.close()
        self.expir.close()
        
    def get_state(self):
        return self._state
    
    def __inc_state(self):
        self._state = (self._state + 1) % 4
    
    def __del__(self):
        self.gpio_cleanup()

    def update(self):
        # Updates valve controller using sensor readings and control settings. 
        # Should also log prior controller commands (not implemented).
    # Control scheme assumes the use of a PEEP valve
        assert(self.inlet.value == True),"Inlet valve should be open!"
        self.logger.update(self._state)
        #self.__print_status()
                        
        # change the following to grabbing last observations from logger
        if(self._state==0):
            # INHALING
            # Inlet flow is open, expiratory dump valve closed
            if(self.logger.data[self.logger.i-1,2] > self.settings.pip):
                self.__hold_pip()
        elif(self._state==1):
            # Maintaining inspiratory plateau.
            # All valves closed.
            elapsed = time.time() - self.start_time
            if(elapsed > self.settings.insptime):
                self.__exhale()
        elif(self._state==2):
            # EXHALING
            # Open expiratory valve and dump pressure.
            if(self.logger.data[self.logger.i-1,2] < self.settings.peep): 
                self.__hold_peep()
        elif(self._state==3):
            # AT REST 
            # Open inspiratory valve and maintain PEEP level w/PEEP valve.
            elapsed = time.time() - self.start_time
            if(elapsed > 60/self.settings.breathrate):
                self.__inhale()
        
    def __inhale(self):
        self.start_time = time.time()
        self.inspir.on()
        self.expir.off()
        self.__inc_state()
        self.__print_status()
        
    def __hold_pip(self):
        self.inspir.off()
        self.__inc_state()
        self.__print_status()
        
    def __exhale(self):
        self.expir.on()
        #self.inspir.on()
        self.__inc_state()
        self.__print_status()
        
    def __hold_peep(self):
        self.expir.off()
        #self.inspir.on()
        self.logger.calculate_last_breath()
        #self.__print_breath()
        self.__inc_state()
        self.__print_status()
        
    def __print_status(self):
        # change this to grabbing last observations from logger
        print("STATE: %2d pressure_0: %4.1f pressure_1: %4.1f Temp: %4.1f flow: %4.0f"%(self._state,self.logger.data[self.logger.i-1,2],self.logger.data[self.logger.i-1,3],self.jp.get_temperature(),self.logger.data[self.logger.i-1,5]))#,end="\r")

    def __print_breath(self,tv):
        return
        # TODO -------------------------------------------------
        #print("\n Last breath: %2.1f seconds, %4.0f mL tidal volume"%(0,0))

class DataLogger:
# A class for keeping track of prior sensor values.
# These are used by controllers and to sound alarms.
    def __init__(self, jp):
        self._init_time         = time.time()
        self.jp                 = jp
        self._track_len         = 1000    # Type int: How long to retain sensor data. Could vary this later per sensor.
        self.n_observations     = 0
        self._breath_ticks      = 0
        self.i                  = 0
        self.data               = np.zeros((self._track_len,10),dtype=np.float32)
        
# By-tick observations
        # data[:,0] is time elapsed
        # data[:,1] is controller state
        # data[:,2] is pressure 0
        # data[:,3] is pressure 1
        # data[:,4] is o2
        # data[:,5] is flow
        # data[:,6] is temperature
        # data[:,7] is humidity
        
# By-breath metrics
        # data[:,8] is breath time
        # data[:,9] is tidal volume
               
    def update(self,state):
    # Takes in current sensor readings object; updates tracking log for all sensors.
        self.data[self.i,:8] = [time.time()-self._init_time,state,self.jp.get_pressure(0),self.jp.get_pressure(1),self.jp.get_o2(),self.jp.get_flow(),self.jp.get_temperature(),self.jp.get_humidity()]
        self._breath_ticks      = self._breath_ticks + 1 
        self.i = (self.i + 1)%self._track_len

    def calculate_last_breath(self):
        # Calculate length of last breath
        self.data[self.i,8] = self.data[self.i-1,0]-self.data[self.i-self._breath_ticks,0]
        # integrate flow over the last breath
        for breath in range(self._breath_ticks):
                # Sum ( ( flow per min / 60s per min )*(time elapsed - last time elapsed) )
                self.data[self.i,9] = self.data[self.i,9] + (self.data[self.i-breath,5]/60)*(self.data[self.i-breath,0]-self.data[self.i-breath-1,0])*1000
        # reset _breath_ticks
        print("Last breath: %4.2f seconds, %4.2f mL tidal volume"%(self.data[self.i,8],self.data[self.i,9]))
        self._breath_ticks = 0
    
