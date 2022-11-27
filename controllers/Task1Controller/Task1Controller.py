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
        self.max_speed = 6  # m/s
        self.turn_speed = 2.3 # m/s
        self.no_of_turns = 1 # Number of corners it is to turn
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
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
        
        # Flag to hold if the black square has been detected.
        self.flag_blackSqaure = False
        
    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self, turn_counter):
          
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collisions
            # Collect data from the left sensors if turning left, right if turning right
            if(self.flag_blackSqaure == False):
                wallDetections = np.max(self.inputs[3:11])
            elif (self.flag_blackSqaure == True):
                wallDetections = np.max(self.inputs[0:3])
                
            # If walls are detected
            if(wallDetections > 0.4):
                # Inc turn counter
                turn_counter = turn_counter + 1
                # Time
                time = datetime.now()
                print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
                # Stop the robot
                self.velocity_left = 0;
                self.velocity_right = 0;
                # If this turn will be more than the number of times it has already turned then don't do anything
                if(turn_counter > self.no_of_turns):
                    print("Sim ends at: {}-{}".format(time.second, time.microsecond))
                else:
                    # If it has not turned enough and has not found the black square then it turns left
                    if(self.flag_blackSqaure == False):
                         print("Turning left at time: {}-{}".format(time.second, time.microsecond))
                         self.velocity_left = -abs(self.turn_speed);
                         self.velocity_right = self.turn_speed;
                    else:
                        # If it has found the black square then it turns right
                        print("Turning right at time: {}-{}".format(time.second, time.microsecond))
                        self.velocity_left = self.turn_speed;
                        self.velocity_right = -abs(self.turn_speed);
            else:
               # If no wall is detected travel at max speed
               time = datetime.now()
               print("Turn complete at: {}-{}".format(time.second, time.microsecond))
               self.velocity_left = self.max_speed;
               self.velocity_right = self.max_speed; 
               
        # Set the motors to the newly assinged speed 
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        return turn_counter

    def run_robot(self):        
        # Main Loop
        # Counter that increases every cycle of the main loop
        count = 0;
        # Counter that holds the number of turns the robot has made
        turn_counter = 0;
        # Array that holds all the sensor inputs
        inputs_avg = []
        # Sets the robots motors to the set speed, making it begin moving
        self.velocity_left = self.max_speed;
        self.velocity_right = self.max_speed;
        # Infinite loop
        time = datetime.now()
        print("Start time is {}-{}".format(time.second, time.microsecond))
        while self.robot.step(self.time_step) != -1 and (turn_counter <= self.no_of_turns):
            self.inputs = []
            # Read Ground Sensors
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            # Check for Black Square on Ground
            if(center < 500 and self.flag_blackSqaure == False):
                self.flag_blackSqaure = True
                print("Black Square Detected")
            # Read Distance Sensors
            for i in range(8):
                # Don't include the two back most sensors, 3 and 4
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
      
            smooth = 30
            # Smooth filter (Average)
            if(count == smooth):
                # Every 30th cycle compile the sensor inputs
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # And run the function to Compute and actuate
                turn_counter = self.sense_compute_and_actuate(turn_counter)
                # Reset counter and empty inputs_avg array
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                # If not the 30th cycle incerease the counter
                inputs_avg.append(self.inputs)
                count = count + 1
                
            
if __name__ == "__main__":
    # Creates a new robot
    my_robot = Robot()
    # Creates a new controller with the new robot
    controller = Controller(my_robot)
    # Runs the main loop with the new controller
    controller.run_robot()
    
