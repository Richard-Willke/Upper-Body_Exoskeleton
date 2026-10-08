import multiprocessing
import numpy as np
import pandas as pd
import time

import position_controller #used for the activity triggered assistive control
import force_controller    #used for the continuous assistive control

class assistiveControl: # redundant and no longer being used. still kept in for completioness sake 
    def __init__(self):
        self.t_rise = 4.0
        self.T = 0.0
        self.supportiveness = 0.5
        self.rectangle_rise = lambda f,t : min(f * t / self.t_rise, f) * self.supportiveness
        self.rectangle_fall = lambda f,t : max(min(f * (1 - (t / self.t_rise)), f), 0) * self.supportiveness
        self.line = [0,0,0,0]
        self.rising_edge1 = True
        self.rising_edge2 = True
        self.t_int1 = 0.0
        self.t_int2 = 0.0

    def assist_loop(self,data,t_delta): 
        #t_delta = (pd.Timestamp.now() - t).total_seconds()
        #t = pd.Timestamp.now()

        if data[0] - data[1] == 0:
            self.t_int1 = 0.0
            self.rising_edge1 = False
        else: 
            self.t_int1 = self.t_int1 + t_delta
        
        if data[2] - data[3] == 0:
            self.t_int2 = 0.0
            self.rising_edge2 = False
        else: 
            self.t_int2 = self.t_int2 + t_delta   

        if self.rising_edge1 == True:
            f1 = int(self.rectangle_rise(data[0], self.t_int1))
            f2 = int(self.rectangle_rise(data[1], self.t_int1))
        
        else: 
            f1 = int(self.rectangle_fall(data[0], self.t_int1))
            f2 = int(self.rectangle_fall(data[1], self.t_int1))
            

        if self.rising_edge2 == True:
            f3 = int(self.rectangle_rise(data[2], self.t_int2))
            f4 = int(self.rectangle_rise(data[3], self.t_int2))
        
        else:
            f3 = int(self.rectangle_fall(data[2], self.t_int2))
            f4 = int(self.rectangle_fall(data[3], self.t_int2))

        self.line = [f1,f2,f3,f4]
        #print(self.t_int1, self.t_int2)
        return self.line


class triggeredControl: # activity triggered control
    """
    Manages activity-triggered control, switching between force and position control based on trigger flags.
    """
    def __init__(self): #most of the parameters are redundant and no longer being used. the only important ones left are the trigger flags 
        """
        Initializes control parameters, primarily focusing on trigger flags for control logic.
        """
        self.k_v = lambda v: v/self.v_min

        ### Unused parameters. can be removed 
        # Define Dynamic Exo Params
        self.L2 = 0.32  
        self.L3 = 0.02
        self.b = 1  
        self.k1 = 10  
        self.k2 = 5 
        self.theta1 = 0.523599  

        self.m2 = 10
        self.I111 = 0.4  
        self.I112 = 0.2
        self.I113 = 0.2
        self.I122 = 0.4
        self.I123 = 0.2
        self.I133 = 0.2

        self.I211 = 1  
        self.I212 = 0.5
        self.I213 = 0.5
        self.I222 = 1
        self.I223 = 0.5
        self.I233 = 0.5

        self.g = -9.81
        self.gx = 9.81
        self.gy = 0
        self.gz = 0

        self.timeEnd = 2
        self.qEnd = np.matrix([[self.deg2rad(160), self.deg2rad(180)], [0.0, 0.0], [0.0, 0.0]])

        self.posControl = position_controller.posCtrl(self.L2, self.m2, self.b, self.k1, self.theta1,
                                                      self.I111, self.I112, self.I113, self.I122, self.I123,
                                                      self.I133, self.gx, self.gy)

        self.posControl.init(self.qEnd, self.timeEnd)

        self.qStart = np.array([0.0, 0.0, 0.0])
        self.qEnd = np.array([[0.0,0.0], [0.0,0.0], [0.0,0.0]])
        self.timeEnd = 5.0
        ###

        self.trigger = False            # trigger used to determine whether to switch from force to position control
        self.return_trigger = False     # trigger used to determine whether the moving up or moving down position controller should be used in case also the trigger flag is set 

    def deg2rad(self, deg):
        return deg * np.pi / 180

    def assist_loop2(self,avg1,avg2,q1,q2):
        """
        Determines control mode based on average forces and joint positions.

        Parameters:
            avg1 (float): Average force for the first joint.
            avg2 (float): Average force for the second joint.
            q1 (float): Current position of the first joint.
            q2 (float): Current position of the second joint.
        """

        if (abs(avg1) <= 1 and (q1 < self.deg2rad(180) and q1 > 0)) or (abs(avg2) <= 1 and (q2 < self.deg2rad(130) and q2 > self.deg2rad(10))): # if the user is no longer physically able to move using their own force, turn on position control
            self.trigger = True 
        else: 
            self.trigger = False

        if abs(q1 - self.deg2rad(180.0)) <= self.deg2rad(3)  and abs(q2 - self.deg2rad(130.0)) <= self.deg2rad(3): # if the maximum position within a certain range has been reached, turn on the return flag as well 
            self.return_trigger = True

        if abs(q1 -0) <= self.deg2rad(3) and abs(q2-self.deg2rad(10)) <= self.deg2rad(3):
            self.return_trigger = False
            self.trigger = False 


#main class used for the control of the exoskeleton
class exoControl:
    """
    Main class for controlling the exoskeleton, managing both continuous and activity-triggered assistive control.
    """
    def __init__(self, incoming_data_rcv, outgoing_data_send):
        """
        Initializes the exoskeleton control with communication pipes and dynamic parameters.
        """
        self.incoming_data_rcv = incoming_data_rcv                      # data received by the serial pipeline
        self.outgoing_data_send = outgoing_data_send                    # data that is sent back to the serial pipeline

        # Define Loop Params 
        self.rate = 100  # loop rate in [Hz]
        self.delta_t = 1 / self.rate 
        print("Control loop rate: " + str(self.rate) + " Hz" + " Delta t: " + str(self.delta_t))

        # Define Dynamic Exo Params
        self.L2 = 0.32                                                  # lower arm length reaching from elbow to wrist
        self.L3 = 0.02                                                  # length from wrist to center of palm
        self.b = 1                                                      # This param can be confirmed in literature and slighly changed to match a more natural arm
        self.k1 = 5                                                     # This param can be confirmed in literature and slighly changed to match a more natural arm
        self.k2 = 20
        self.theta1 = 0.523599                                          # not used 

        self.m2 = 10                                                    # mass of the lower arm. Value chosen arbitrarily 
        self.I111 = 0.4                                                 # Inertial values of the arm. First index means the arm piece it is coresponding to (1 = upper arm, 2 = lower arm, 3 = hand)
        self.I112 = 0.2                                                 # second and third index are the corresponding axis if each arm piece were to be approximated by a cuboid 
        self.I113 = 0.2                                                 # values are chosen arbitrarily for all Ixxx
        self.I122 = 0.4
        self.I123 = 0.2
        self.I133 = 0.2

        self.I211 = 1                                                   # This param can be confirmed in literature and slighly changed to match a more natural arm
        self.I212 = 0.5
        self.I213 = 0.5
        self.I222 = 1
        self.I223 = 0.5
        self.I233 = 0.5

        self.g = -9.81

        # init params 
        self.tao = np.atleast_2d(np.array([0.0, 0.0])).T              # acceleration on the individual joints of the robot arm

        self.q1 = self.deg2rad(0)                                     # angle of wrist (pronation, supination)
        self.qd1 = self.deg2rad(0)                                    # angular speed of wrist 
        self.qdd1 = self.deg2rad(0)                                   # angular acceleration of wrist

        self.q2 = self.deg2rad(0)                                     # angle of elbow (flexion, extension)
        self.qd2 = self.deg2rad(0)                                    # angular speed of elbow joint
        self.qdd2 = self.deg2rad(0)                                   # angular acceleration of elbow joint

        self.q = np.atleast_2d(np.array([self.q1, self.q2])).T        # vector of all joint angles
        self.qd = np.atleast_2d(np.array([self.qd1, self.qd2])).T     # vector of all joint angular speeds 
        self.qdd = np.atleast_2d(np.array([self.qdd1, self.qdd2])).T  # vector of all joint angular accelerations

        self.gx = 9.81                                                # graitational force. Initially modeled as only going along the x-axis 
        self.gy = 0
        self.gz = 0

        self.timeEnd = 10                                                                                   # 10 seconds for the positional controller's trajectory to reach the end
        self.qEnd_up = np.matrix([[self.deg2rad(180), self.deg2rad(130)], [0.0, 0.0], [0.0, 0.0]])          # goal angles as well as velocities and accelerations for the position controller
        self.qEnd_down = np.matrix([[self.deg2rad(0), self.deg2rad(10)], [0.0, 0.0], [0.0, 0.0]])           # up and down being the goal states for the moving up and moving down actions of the human arm during the activity triggered assistance 

        self.m_matrix = np.matrix([[0,0],[0,0]])                                                            # matrices used for the system dynamics. M = Intertia Matrix, C = Coriolis Matrix, G = Gravitational acceleration vector, B = Damping matrix 
        self.c_matrix = np.matrix([[0,0],[0,0]])
        self.g_matrix = np.atleast_2d(np.array([0,0])).T
        self.b_matrix = np.matrix([[self.b, 0], [0, self.b]])

        self.posControl_up = position_controller.posCtrl(self.L2, self.m2, self.b, self.k1, self.theta1,       # position controller, used for the moving up action
                                                      self.I111, self.I112, self.I113, self.I122, self.I123,
                                                      self.I133, self.gx, self.gy)

        self.posControl_up.init(self.qEnd_up, self.timeEnd)                                                    # initialization of the position controller (up)

        self.posControl_down = position_controller.posCtrl(self.L2, self.m2, self.b, self.k1, self.theta1,     # position controller, used for the moving down action
                                                      self.I111, self.I112, self.I113, self.I122, self.I123,
                                                      self.I133, self.gx, self.gy)

        self.posControl_down.init(self.qEnd_down, self.timeEnd)                                                # initialization of the position controller (down)

        self.forceControl = force_controller.forceCtrl(self.k1, self.k2, self.L2,self.L3)                      # force controller, used for both assistive scenarios
        self.W_des = 0.0                                                                                       # desired goal wrench, that is applied onto the system
        self.Ws1 = 0.0                                                                                         # wrench measured by force sensors 1 and 2 (corresponding to data[0] and data[1])
        self.Ws2 = 0.0                                                                                         # wrench measured by force sensors 3 and 4 (corresponding to data[2] and data[3])
        self.forceControl.init(self.W_des)                                                                     # initialization of the force controller 

        #self.assist = assistiveControl()                                                                      # continuous assistive control, using only the force control. Instead of being its own class, it has been implemented as the standard scenario in the exo control, for simplicity
        self.triggered = triggeredControl()                                                                    # activity triggered control, using a mixture of both force control and position control

        self.W1_avg = [0,0,0,0,0]                                                                              # samples of the last 5 applied wrenches, the average of these values is used to determine the mode of the activity triggered assistance 
        self.W2_avg = [0,0,0,0,0]                                                                              # samples of the last 5 applied wrenches, the average of these values is used to determine the mode of the activity triggered assistance 
        self.assist_mode = False #True                                                                         # current mode of the system. False = Continuous assistance, True = Activity Triggered assistance; is controlled by 
        #self.force_control = True                                                                              
        self.avg1 = 2000                                                                                       # average values of the 5 last applied wrenches
        self.avg2 = 2000

        self.calibration = True                                                                                # indexing, whether system is currently calibrating. The calibration is only done once the program is started
        self.sensor_offsets = np.zeros((100,4))                                                                # values used during calibration to measure the offset force that is applied on the sensors
        self.iteration = 0                                                                                     
        self.offset = np.zeros(4)                                                                              # offset values. Used outside of the calibration procedure to eliminate  any false sensor readings

    def deg2rad(self, deg):
        return deg * np.pi / 180

    def rad2deg(self, rad):
        return rad * 180 / np.pi

    def checkRange(self):
        """
        Limits the range of motion for each joint, adjusting angles, velocities, and accelerations as needed.
        """
        # function used to limit the RoM of each individual joint. THe joints are set to their max / min values once they would go over the boundaries of the RoM. additionally, angular velocity and acceleration are set to zero in this case
        if (self.q[0,0] > self.deg2rad(180)):
            self.q[0,0] = self.deg2rad(180)
            self.qd[0,0] = 0
            self.qdd[0,0] = 0

        elif (self.q[0,0] < self.deg2rad(0)):
            self.q[0,0] = self.deg2rad(0)
            self.qd[0,0] = 0
            self.qdd[0,0] = 0

        if (self.q[1,0] > self.deg2rad(130)):
            self.q[1,0] = self.deg2rad(130)
            self.qd[1,0] = 0
            self.qdd[1,0] = 0
        elif (self.q[1,0] < self.deg2rad(10)):
            self.q[1,0] = self.deg2rad(10)
            self.qd[1,0] = 0
            self.qdd[1,0] = 0

    def control_loop(self):
        """
        Main control loop for the exoskeleton, handling calibration, force and position control.
        """
        t = pd.Timestamp.now()
        t_int = 0
        while self.calibration == True:                                         # calibration routine, to calculate the offsets of the force sensor readings. the user shall not move during this time and should leave their arm hanging on their side
            if self.iteration < 100:
                try:
                    while (self.incoming_data_rcv.poll()):                      # every time a new sensor reading batch is being received by the serial pipeline, read it and add it to the offset readings
                        data = self.incoming_data_rcv.recv()
                        
                        f1 = data[1]/4096 # normalization
                        f2 = data[0]/4096 # normalization
                        f3 = data[2]/4096 # normalization
                        f4 = data[3]/4096 # normalization

                        self.sensor_offsets[self.iteration,0] = f1
                        self.sensor_offsets[self.iteration,1] = f2
                        self.sensor_offsets[self.iteration,2] = f3
                        self.sensor_offsets[self.iteration,3] = f4
                        self.iteration += 1
                except:                                                         # if faulty values received -> skip and try to read the next batch
                    print("skipping")
            else:
                self.offset = np.sum(self.sensor_offsets, axis = 0)/100         # once the (here: 100) necessary values have been read -> calculate mean of the values and exit the calibration procedure
                print(self.offset)
                print(self.sensor_offsets)
                self.calibration = False
        

        while self.calibration == False:                                        # if the calibration is finished, enter the actual control loop
            try:
                while (self.incoming_data_rcv.poll()):                          # read the incoming data from the serial pipeline
                    data = self.incoming_data_rcv.recv()
                    
                    f1 = data[1]/4096 #- self.offset[0]                         # normalize the data from [0, 4096] to [0, 1], then subtract offset. because of issues with the offset being subtracted on the forces for the pronation / suppination (f1 & f2), it has been omitted for this joint
                    f2 = data[0]/4096 #- self.offset[1]                         
                    f3 = data[2]/4096 - self.offset[2]                          # f3 & f4 for elbow flexion and extension 
                    f4 = data[3]/4096 - self.offset[3]                          

                    self.assist_mode = data[4]                                  # hardware switch on the board, digital sensor readings. Used to determine whether continuous assistance or activity triggered assistance is being used 

                    forces = np.array([f1,f2,f3,f4])*4096                       # total forces used to calculate the applied wrenches on the system

                    self.Ws1 = (f1-f2)*5                                        # suggest maximal force
                    self.Ws2 = (f3-f4)*5

                    #fixing issue with one joint being affected by other joint wrench. If one of the wrenches would result in one of the joints going over their limits in RoM, it is set to zero, using max / min functions
                    if self.q1 >= self.deg2rad(180):
                        self.Ws1 = min(0.0,self.Ws1)
                    elif self.q1 <= self.deg2rad(0):
                        self.Ws1 = max(0.0,self.Ws1)

                    if self.q2 >= self.deg2rad(130):
                        self.Ws2 = min(0.0,self.Ws2)
                    elif self.q2 <= self.deg2rad(10):
                        self.Ws2 = max(0.0,self.Ws2) 

                    # calculate the matrices used for the dynamic equations 
                    self.m_matrix = np.matrix([[self.I133 + (pow(self.L2,2)*self.m2)/4 + self.I222*pow(np.cos(self.q2),2) + self.I211*pow(np.sin(self.q2),2) + 2*self.I212*np.cos(self.q2)*np.sin(self.q2), - self.I223*np.cos(self.q2) - self.I213*np.sin(self.q2)],
                                                [- self.I223*np.cos(self.q2) - self.I213*np.sin(self.q2), self.I233]])
                    self.c_matrix = np.matrix([[self.qd2*(self.I212*np.cos(2*self.q2) + (self.I211*np.sin(2*self.q2))/2 - (self.I222*np.sin(2*self.q2))/2), self.qd1*(self.I212*pow(np.cos(self.q2),2) - self.I212*pow(np.sin(self.q2),2) + self.I211*np.cos(self.q2)*np.sin(self.q2) - self.I222*np.cos(self.q2)*np.sin(self.q2)) - self.qd2*(self.I213*np.cos(self.q2) - self.I223*np.sin(self.q2))],
                                            [-self.qd1*(self.I212*np.cos(2*self.q2) + (self.I211*np.sin(2*self.q2))/2 - (self.I222*np.sin(2*self.q2))/2), 0]])
                    self.g_matrix = np.atleast_2d(np.array([-(self.L2*self.m2*(self.gx*np.cos(self.q1) + self.gy*np.sin(self.q1)))/2, 0])).T

                    if self.assist_mode:                                                        # continuous assistance mode 
                        self.tao = self.forceControl.update(self.Ws1,self.Ws2) + self.g_matrix  # calculate accelerations of the joints by the controller 

                    elif not self.assist_mode:                                                  # activity triggered assistance mode 
                        self.W1_avg.append(self.Ws1)
                        self.W2_avg.append(self.Ws2)

                        self.W1_avg.pop(0)
                        self.W2_avg.pop(0)

                        self.avg1 = np.sum(self.W1_avg)/len(self.W1_avg)                        # calculate average values of both wrenches 
                        self.avg2 = np.sum(self.W2_avg)/len(self.W2_avg)
                        self.triggered.assist_loop2(self.avg1, self.avg2, self.q1, self.q2)     # if either one of the two wrenches is smaller than the threshold values, then switch from force driven control to position control for the upwards motion

                        if self.triggered.trigger and not self.triggered.return_trigger:        # if the arm has already reached the goal position for the upwards motion, use the position controller for the downwards motion into the original position
                            self.tao = self.posControl_up.update(self.delta_t, self.q, self.qd, self.qdd, self.m_matrix, self.c_matrix, self.b_matrix, self.g_matrix)   
                        
                        elif self.triggered.return_trigger:
                            self.tao = self.posControl_down.update(self.delta_t, self.q, self.qd, self.qdd, self.m_matrix, self.c_matrix, self.b_matrix, self.g_matrix)

                        else:                                                                   # if the position control isnt triggered, remain using the force controller 
                            f1 = data[0]/4096 # normalization
                            f2 = data[1]/4096 # normalization
                            f3 = data[2]/4096 # normalization
                            f4 = data[3]/4096 # normalization

                            self.Ws1 = (f1-f2)*5 # suggest maximal force
                            self.Ws2 = (f3-f4)*5

                            self.tao = self.forceControl.update(self.Ws1,self.Ws2) + self.g_matrix

                    self.last_tao = self.tao       # if either tao is smaller than a minimum value, then set it to zero. this has worked to reduce the influence of one joint on the other joint if only one is being actuated. the error seens to be coming from a lack of precission from the sensor readings 
                    if abs(self.tao)[0,0] < 0.05:
                        self.tao[0,0] = 0 
                    if abs(self.tao)[1,0] < 0.05:
                        self.tao[1,0] = 0

                    # calculate qdd and integrate
                    t_delta = (pd.Timestamp.now() - t).total_seconds()
                    t = pd.Timestamp.now()
                    t_int = t_int + t_delta
                    self.qdd = np.matmul(np.linalg.inv(self.m_matrix),(self.tao - np.matmul(self.b_matrix,self.qd) - np.matmul(self.c_matrix,self.qd) - self.g_matrix))
                    
                    # reduces the precission effect as discussed before (L.357) (also reduce the dynamic model)
                    if self.tao[0] == self.g_matrix[0]:
                        self.qdd[0] = 0
                    if self.tao[1] == self.g_matrix[1]:
                        self.qdd[1] = 0

                    self.qd = self.qd + t_delta * self.qdd #calculate angular velocities 

                    # reduces the precission effect as discussed before (L.357) (also reduce the dynamic model)
                    if self.tao[0] == self.g_matrix[0]: 
                        self.qd[0] = 0
                    if self.tao[1] == self.g_matrix[1]:
                        self.qd[1] = 0

                    self.q = self.q + t_delta * self.qd # calculate angules

                    self.checkRange()                   # check range of all angles before sending to serial port
                    self.q1 = self.q[0,0]               #
                    self.qd1 = self.qd[0,0]
                    self.qdd1 = self.qdd[0,0]

                    self.q2 = self.q[1,0]
                    self.qd2 = self.qd[1,0]
                    self.qdd2 = self.qdd[1,0]

                    # send data through serial pipeline to serial port. sent data are the twi sensor readings 
                    outgoing_data = str(self.q1) + ',' + str(self.q2) + '\n'
                    self.outgoing_data_send.send(outgoing_data.encode(encoding='UTF-8'))

                    print(f"q1: {self.rad2deg(self.q1):.1f} q2: {self.rad2deg(self.q2):.1f} tau1: {self.tao[0,0]:.2f} tau2: {self.tao[1,0]:.2f} data: {forces} trigger: {self.triggered.trigger} return: {self.triggered.return_trigger} mode: {self.assist_mode}")

                # compensate waiting time
                time.sleep(self.delta_t)
            except KeyboardInterrupt:                                           # if a keyboard interrupt happens, close the control loop
                print("closing control loop")

if __name__ == '__main__':
    # receive data pipe
    incoming_data_rcv, incoming_data_send = multiprocessing.Pipe()
    # send commands pipe
    outgoing_data_rcv, outgoing_data_send = multiprocessing.Pipe()

    exo_control = exoControl(incoming_data_rcv, outgoing_data_send)
    exo_control.control_loop()

