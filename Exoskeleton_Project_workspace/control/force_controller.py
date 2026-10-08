import numpy as np
import pandas as pd

class forceCtrl:
    """
    Implements a force control strategy for an assistive device, using a proportional controller to adjust forces applied by the device based on desired wrench values.
    """
    def __init__(self,k1,k2,L2,L3):
        """
        Initializes the force controller with specific gains and physical dimensions of the assistive device.

        Parameters:
            k1 (float): Proportional gain for the first joint.
            k2 (float): Proportional gain for the second joint.
            L2 (float): Length of the lower arm (from elbow to wrist).
            L3 (float): Distance from wrist to middle of palm.
        """
        print("Initializing force controller")
        self.kp = np.array([[k1,0.0],[0.0,k2]])             # controller gain
        self.L2 = L2                                        # length of lower arm
        self.L3 = L3                                        # distance wrist to middle of palm
        self.startFlag = False
        self.tao = np.atleast_2d(np.array([0.0,0.0])).T
        
    
    def init(self, W_des):
        """
        Initializes or resets the desired wrench for the control loop.

        Parameters:
            W_des (float): Desired wrench value to achieve.

        Returns:
            bool: True to indicate successful initialization.
        """
        self.W_des = W_des      # desired wrench
        self.startFlag = False  
        return True
    
    def update(self,Ws1, Ws2):
        """
        Updates the control output based on the current wrenches applied to the system, comparing them to the desired wrench to calculate the necessary adjustments.

        Parameters:
            Ws1 (float): Current wrench applied at the first joint.
            Ws2 (float): Current wrench applied at the second joint.

        Returns:
            np.ndarray: The calculated torque adjustments to apply to the device's joints.
        """
        if(not self.startFlag):
            self.startFlag = True
        delta_Ws = np.atleast_2d(np.array([self.L2 *(Ws1-self.W_des), self.L3 * (Ws2-self.W_des)])).T    # calculate the difference in the wrenches for the closed loop control
        self.tao = np.matmul(self.kp,delta_Ws)                                                           # apply a gain kp onto the difference of the wrenches
        return self.tao                                                                                  # resulting angular acceleration