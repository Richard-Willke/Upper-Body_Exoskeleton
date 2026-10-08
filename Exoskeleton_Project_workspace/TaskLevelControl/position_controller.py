import numpy as np
import pandas as pd


class posCtrl:
    """
    Implements a position controller for an assistive device, utilizing trajectory generation, PD control, and dynamic parameter adaptation for precise movement control.
    """
    def __init__(self, L2, m2, b1, k1, theta1, I311, I312, I313, I322, I323, I333, gx, gy): # initialization of the controller. Input: dynamic parameters of the exoskeleton
        """
        Initializes the position controller with dynamic parameters of the exoskeleton and controller gains.

        Parameters:
            L2 (float): Length of the lower arm.
            m2 (float): Mass of the lower arm with hand.
            b1, k1 (float): Damping and stiffness coefficients.
            theta1 (float): Initial joint angle.
            I311, I312, I313, I322, I323, I333 (float): Inertia components of the arm segments.
            gx, gy (float): Gravitational acceleration components in the x and y directions.
        """
        print("Initializing position controller")

        self.kp = np.array([[5.0, 0], [0, 5.0]])      # P-controller gain
        self.kd = np.array([[2.5, 0], [0, 2.5]])      # D-Controller gain
        self.ki = np.array([[0.0, 0.0], [0.0, 0.0]])  # I-Controller gain

        # parameters of the system (dynamic parameters). for more detailled information, look into exo_control.py
        self.L2 = L2
        self.m2 = m2
        self.b1 = b1
        self.k1 = k1
        self.theta1 = theta1
        self.I311 = I311
        self.I312 = I312
        self.I313 = I313
        self.I322 = I322
        self.I323 = I323
        self.I333 = I333
        self.gx = gx
        self.gy = gy

        # goal angles, angular velocities as well as angular accelerations
        self.q_des = np.atleast_2d(np.array([0.0, 0.0])).T
        self.qd_des = np.atleast_2d(np.array([0.0, 0.0])).T
        self.qdd_des = np.atleast_2d(np.array([0.0, 0.0])).T

        self.tao = np.atleast_2d(np.array([0.0, 0.0])).T
        self.taor = np.atleast_2d(np.array([0.0, 0.0])).T

        self.deltaQ = np.atleast_2d(np.array([0.0, 0.0])).T

        # start time and end time for the trajectory calculation (feed forward controller)
        self.timeStart = pd.Timestamp.now()
        self.qStart = np.array([0.0, 0.0, 0.0])
        self.qEnd = np.array([[0.0,0.0], [0.0,0.0], [0.0,0.0]])
        self.timeEnd = 0.0

        self.startFlag = False

        # for traj gen
        self.aq1 = np.zeros([6, 1])
        self.aq2 = np.zeros([6, 1])
        self.q_traj_1 = np.zeros([6, 1])
        self.q_traj_2 = np.zeros([6, 1])
        self.T_1 = np.zeros([6, 6])
        self.T_2 = np.zeros([6, 6])

        # for repetitive traj gen
        self.qf_amp_1 = 0.0
        self.qf_amp_2 = 0.0
        self.T = 10.0

    def init(self, qEnd, timeEnd): # initialization of the controller 
        """
        Initializes or resets the goal states and the duration for the trajectory.

        Parameters:
            qEnd (np.array): Goal joint states (angles, velocities, accelerations).
            timeEnd (float): Duration to reach the goal state.

        Returns:
            bool: True indicating successful initialization.
        """
        self.qEnd = qEnd
        self.timeEnd = timeEnd
        self.startFlag = False
        return True

    # 3. Trajectory generation
    def trajGen(self, qi, qf, qpi, qpf, qppi, qppf, t0, tf, tc):
        """
        Generates a trajectory for the joint motion using polynomial coefficients.

        Parameters:
            qi, qf (float): Initial and final joint angles.
            qpi, qpf (float): Initial and final joint velocities.
            qppi, qppf (float): Initial and final joint accelerations.
            t0, tf (float): Start and end times for the trajectory.
            tc (float): Current time.
        """
        p = np.zeros([6, 1])
        v = np.zeros([6, 1])
        a = np.zeros([6, 1])

        if not self.startFlag:
            qf_1 = qf[0, 0]
            qf_2 = qf[0, 1]

            qpf_1 = qpf[0, 0]
            qpf_2 = qpf[0, 1]

            qppf_1 = qppf[0, 0]
            qppf_2 = qppf[0, 1]

            self.q_traj_1 = np.array([qi, qf_1, qpi, qpf_1, qppi, qppf_1])
            self.q_traj_2 = np.array([qi, qf_2, qpi, qpf_2, qppi, qppf_2])

            self.T_1 = np.array([[1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
                                 [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
                                 [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
                                 [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
                                 [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
                                 [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]])

            self.T_2 = np.array([[1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
                                 [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
                                 [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
                                 [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
                                 [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
                                 [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]])

            self.aq1 = np.matmul(np.linalg.inv(self.T_1), self.q_traj_1)

            self.aq2 = np.matmul(np.linalg.inv(self.T_2), self.q_traj_2)

        if tc <= tf:
            p = np.array([1, tc, tc ** 2, tc ** 3, tc ** 4, tc ** 5])
            v = np.array([0, 1, 2 * tc, 3 * tc ** 2, 4 * tc ** 3, 5 * tc ** 4])
            a = np.array([0, 0, 2, 6 * tc, 12 * tc ** 2, 20 * tc ** 3])

            q_des_1 = np.matmul(np.transpose(p), self.aq1)
            qd_des_1 = np.matmul(np.transpose(v), self.aq1)
            qdd_des_1 = np.matmul(np.transpose(a), self.aq1)

            q_des_2 = np.matmul(np.transpose(p), self.aq2)
            qd_des_2 = np.matmul(np.transpose(v), self.aq2)
            qdd_des_2 = np.matmul(np.transpose(a), self.aq2)

            self.q_des = np.atleast_2d(np.array([q_des_1, q_des_2])).T
            self.qd_des = np.atleast_2d(np.array([qd_des_1, qd_des_2])).T
            self.qdd_des = np.atleast_2d(np.array([qdd_des_1, qdd_des_2])).T
        else:
            q_des_1 = qf[0, 0]
            qd_des_1 = qpf[0, 0]
            qdd_des_1 = qppf[0, 0]

            q_des_2 = qf[0, 1]
            qd_des_2 = qpf[0, 1]
            qdd_des_2 = qppf[0, 1]

            self.q_des = np.atleast_2d(np.array([q_des_1, q_des_2])).T
            self.qd_des = np.atleast_2d(np.array([qd_des_1, qd_des_2])).T
            self.qdd_des = np.atleast_2d(np.array([qdd_des_1, qdd_des_2])).T

    def Rept_trajGen(self, qi, qf, T, tc):
        """
        Generates a repetitive trajectory for cyclic motions.

        Parameters:
            qi (float): Initial joint angle.
            qf (float): Final joint angle amplitude.
            T (float): Period of the cyclic motion.
            tc (float): Current time within the cycle.
        """
        if not self.startFlag:
            # param init
            self.qf_amp_1 = qf[0, 0]
            self.qf_amp_2 = qf[0, 1]

        q_des_1 = self.qf_amp_1/2*np.sin(2*np.pi/T*tc) + qi + self.qf_amp_1/2
        qd_des_1 = self.qf_amp_1/2*2*np.pi/T*np.cos(2*np.pi/T*tc)
        qdd_des_1 = -self.qf_amp_1/2*np.power((2*np.pi/T),2)*np.sin(2*np.pi/T*tc)

        q_des_2 = self.qf_amp_2/2*np.sin(2*np.pi/T*tc) + qi + self.qf_amp_2/2
        qd_des_2 = self.qf_amp_2/2*2*np.pi/T*np.cos(2*np.pi/T*tc)
        qdd_des_2 = -self.qf_amp_2/2*np.power((2*np.pi/T),2)*np.sin(2*np.pi/T*tc)

        self.q_des = np.atleast_2d(np.array([q_des_1, q_des_2])).T
        self.qd_des = np.atleast_2d(np.array([qd_des_1, qd_des_2])).T
        self.qdd_des = np.atleast_2d(np.array([qdd_des_1, qdd_des_2])).T

    # 5. PD Controller
    def update(self, delta_t, q, qd, qdd, m, c, b, g):
        """
         Updates the control output based on the current state, desired trajectory, and dynamic parameters.

         Parameters:
             delta_t (float): Time step for the update.
             q, qd, qdd (np.array): Current joint states.
             m, c, b, g (np.array): Dynamic parameters matrices.

         Returns:
             np.array: The calculated torque to be applied to the joints.
         """
        if not self.startFlag:
            self.timeStart = pd.Timestamp.now()
            self.trajGen(self.qStart[0], self.qEnd[0], self.qStart[1], self.qEnd[1], self.qStart[2], self.qEnd[2], 0.0, self.timeEnd, 0.0)
            #self.Rept_trajGen(self.qStart[0], self.qEnd[0], self.T, 0.0)
            self.startFlag = True
        else:
            self.trajGen(self.qStart[0], self.qEnd[0], self.qStart[1], self.qEnd[1], self.qStart[2], self.qEnd[2], 0.0, self.timeEnd, (pd.Timestamp.now() - self.timeStart).total_seconds())
            #self.Rept_trajGen(self.qStart[0], self.qEnd[0], self.T, (pd.Timestamp.now() - self.timeStart).total_seconds())

        deltaQ = q - self.q_des
        deltaQd = qd - self.qd_des

        qdr = self.qd_des - np.matmul(self.kp, deltaQ)
        qddr = self.qdd_des - np.matmul(self.kp, deltaQd)

        Sq = qd - qdr

        self.tao = -np.matmul(self.kd, Sq) + np.matmul(m, qddr) + np.matmul(c, qdr) + g

        return self.tao
