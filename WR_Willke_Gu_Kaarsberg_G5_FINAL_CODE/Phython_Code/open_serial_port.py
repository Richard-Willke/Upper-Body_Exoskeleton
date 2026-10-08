# pip install pyserial
import serial 
import multiprocessing

class serialCom:
    """
    Manages communication between the exoskeleton control system and the ESP32 board via serial port, facilitating data exchange for control and feedback.
    """
    def __init__(self,port, baud_rate,incoming_data_send=multiprocessing.Pipe(),
                 outgoing_data_rcv=multiprocessing.Pipe(),enable_multiprocessing=True):         # initialize serialCom obejct by establishing a pipeline between the exo control and the ESP32 board using the serial port
        """
        Initializes serial communication with specified port and baud rate, along with setting up multiprocessing communication pipes.

        Parameters:
            port (str): Serial port to use for communication.
            baud_rate (int): Baud rate for the serial communication.
            incoming_data_send (multiprocessing.Pipe, optional): Sending end of a pipe for incoming data from the ESP32.
            outgoing_data_rcv (multiprocessing.Pipe, optional): Receiving end of a pipe for outgoing data to the ESP32.
            enable_multiprocessing (bool, optional): Enables or disables multiprocessing features.
        """
        print("Port connection: " + port + " Baud rate: " + str(baud_rate))
        self.source = serial.Serial(port=port, baudrate=baud_rate, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        self.incoming_data_send = incoming_data_send
        self.outgoing_data_rcv = outgoing_data_rcv
        self.enable_multiprocessing = enable_multiprocessing

    def serial_loop(self):
        """
        Continuously reads from and writes to the serial port, handling data exchange between the control system and the ESP32. Decodes incoming messages, checks format, converts to float values, and sends them through a pipe. Also, sends commands received from the control system to the ESP32.
        """
        while True:
            try:
                if self.enable_multiprocessing:
                    line = self.source.readline().decode('utf-8')                               # receive the input from the ESP32 and decode it
                    line = line.strip()
                    if line.startswith("<") and line.endswith(">"):                             # split the string of data into an array consisting of 5 elements [force1, force2, force 3, force4, control_mode]
                        values = line[1:-1].split(',')
                        try:
                            self.line = [float(item) for item in values]
                            #print(self.line)
                        except ValueError as e:
                            print("Error converting values:", e)
                    else:
                        self.line = [0,0,0,0]
                    
                    self.incoming_data_send.send(self.line)                                     # send the data to the exo control

                    while self.outgoing_data_rcv.poll():                                    
                        self.source.write(self.outgoing_data_rcv.recv())                        # send the received angle values received from the exo control to the ESP32
                        #print(self.outgoing_data_rcv.recv())
                else:
                        self.source.write(b"1.0 \n")
                        
            except KeyboardInterrupt:                                                           # close the loop if a keyboard interrup happens 
                print("closing serial loop")
                #self.source.close()
 
if __name__ == '__main__':
    port = "COM8"
    baud_rate = 9600
    serialCom = serialCom(port, baud_rate,enable_multiprocessing=False)
    serialCom.serial_loop()
    