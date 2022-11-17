from controller import Robot
from datetime import datetime
import math
import numpy as np
import threading


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
        
        # Flag to hold if the black square has been detected.
        self.blackSqaure = False
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)
            
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag
        self.flag_turn = 0
        
    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self, turn_counter):
          
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collision
            
            if(self.blackSqaure == False):
                wallDetections = np.max(self.inputs[3:11])
            elif (self.blackSqaure == True):
                wallDetections = np.max(self.inputs[0:3])
       
            if(wallDetections > 0.4):
                # Time
                time = datetime.now()
                turn_counter = turn_counter + 1
                if(turn_counter == 2):
                    print("END OF SIM")
                print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
                self.velocity_left = -1;
                self.velocity_right = -1;
                self.flag_turn = 1
                #return turn_counter;
            # Turn
            if(self.flag_turn):
                if(self.blackSqaure == False):
                    print("Turning Left")
                    self.velocity_left = -2.3;
                    self.velocity_right = 2.3;
                    if(np.min(self.inputs[0:3])< 0.35):
                        self.flag_turn = 0
                else:
                    print("Turning Right")
                    self.velocity_left = 2.3;
                    self.velocity_right = -2.3;
                    if(np.min(self.inputs[0:3])< 0.35):
                        self.flag_turn = 0
            else:        
                self.velocity_left = 4;
                self.velocity_right = 4;

     
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        

    def run_robot(self):        
        # Main Loop
        count = 0;
        turn_counter = 0;
        inputs_avg = []
        self.velocity_left = 4;
        self.velocity_right = 4;
        while self.robot.step(self.time_step) != -1:
            # Read Ground Sensors
            self.inputs = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            
            if(center < 500 and self.blackSqaure == False):
                self.blackSqaure = True
                print("Black Square Detected")
            
            # Read Distance Sensors
            for i in range(8):
                if(i==0 or i==1 or i==2 or i==5 or i==6 or i==7):        
                    temp = self.proximity_sensors[i].getValue()
                    # Adjust Values
                    min_ds = 0
                    max_ds = 2400
                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds
                    # Save Data
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    #print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
      
            # Smooth filter (Average)
            smooth = 30
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_and_actuate(turn_counter)
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    
