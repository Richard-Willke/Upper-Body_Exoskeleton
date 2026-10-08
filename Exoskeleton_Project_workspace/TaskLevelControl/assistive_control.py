import multiprocessing
import numpy as np
import pandas as pd
import time

class assistiveControl:
    """
    Manages assistive control through multiprocessing, adjusting data based on signal edges and supportiveness levels.

    Attributes:
        incoming_data_send (Pipe): Sending end of a pipe for incoming data.
        outgoing_data_rcv (Pipe): Receiving end of a pipe for outgoing data.
        enable_multiprocessing (bool): If true, enables multiprocessing.
        t_rise (float): Rise time for signal adjustment.
        supportiveness (float): Adjustment factor for signal processing.
        rectangle_rise (function): Calculates rising edge signal value.
        rectangle_fall (function): Calculates falling edge signal value.
        line (list): Stores the latest signal values.
        rising_edge (bool): Indicates if the signal is in the rising edge phase.
    """
    def __init__(self, incoming_data_send = multiprocessing.Pipe(), outgoing_data_rcv = multiprocessing.Pipe(), enable_multiprocessing = True):
        """
        Initializes the assistive control system.
        """
        self.incoming_data_send = incoming_data_send
        self.outgoing_data_rcv = outgoing_data_rcv
        self.enable_multiprocessing = enable_multiprocessing
        self.t_rise = 2.0
        self.T = 0.0
        self.supportiveness = 0.5
        self.rectangle_rise = lambda f,t : min(f * t / self.t_rise, f) * self.supportiveness
        self.rectangle_fall = lambda f,t : max(min(f * (1 - t / self.t_rise), f), 0) * self.supportiveness
        self.line = [0,0,0,0]
        self.rising_edge = True


    def assist_loop(self):
        """
        Main loop that adjusts and sends data based on signal supportiveness and edge detection.
        """
        t = pd.Timestamp.now()
        t_int = 0
        while True:
            try:
                while (self.outgoing_data_rcv.poll()): #instead of if
                    data = self.outgoing_data_rcv.recv()
                    
                    t_delta = (pd.Timestamp.now() - t).total_seconds()
                    t = pd.Timestamp.now()
                    

                    if data[0] - data[1] == 0 and data[2] - data[3] == 0:
                        t_int = 0
                        self.rising_edge = False
                    else: 
                        t_int = t_int + t_delta

                    
                    if self.rising_edge == True:
                        f1 = int(self.rectangle_rise(data[0], t_int))
                        f2 = int(self.rectangle_rise(data[1], t_int))
                        f3 = int(self.rectangle_rise(data[2], t_int))
                        f4 = int(self.rectangle_rise(data[3], t_int))
                    else: 
                        f1 = int(self.rectangle_fall(data[0], t_int))
                        f2 = int(self.rectangle_fall(data[1], t_int))
                        f3 = int(self.rectangle_fall(data[2], t_int))
                        f4 = int(self.rectangle_fall(data[3], t_int))

                    self.line = [f1,f2,f3,f4]
                    self.incoming_data_send.send(self.line)
                    #while self.outgoing_data_rcv.poll(): #if self.outgoing_data_rcv.poll():
                    #    self.source.write(self.outgoing_data_rcv.recv())
                    #print(self.line,t_int)

            except KeyboardInterrupt:
                print("closing assistive loop")
                #self.source.close()
 