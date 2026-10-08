import open_serial_port as serialScript
import exo_control as exoControl
import assistive_control as assist
import multiprocessing


def main_serial_process(port,baud_rate,incoming_data_send, outgoing_data_rcv):
    serialCom = serialScript.serialCom(port, baud_rate,incoming_data_send, 
                                       outgoing_data_rcv,enable_multiprocessing=True)
    serialCom.serial_loop()

def main_control_process(incoming_data_rcv, outgoing_data_send):
    exoCtrl = exoControl.exoControl(incoming_data_rcv, outgoing_data_send)
    # exoCtrl.test() 
    exoCtrl.control_loop() # TO DO: comment test and uncomment this line to work w controller

def main_assist_process(incoming_data_rcv, outgoing_data_send):
    assistive = assist.assistiveControl(incoming_data_rcv, outgoing_data_send)
    # exoCtrl.test() 
    assistive.assist_loop() # TO DO: comment test and uncomment this line to work w controller


if __name__ == '__main__':
    port = "COM4" #"/dev/cu.usbserial-02479C5E"
    baud_rate = 9600

    # receive data pipe
    incoming_data_rcv, incoming_data_send = multiprocessing.Pipe()

    # send commands pipe
    outgoing_data_rcv, outgoing_data_send = multiprocessing.Pipe()

    # send assistive forces pipe
    force_data_rcv, force_data_send = multiprocessing.Pipe()

    # serial process 
    p_serial = multiprocessing.Process(target=main_serial_process, args=(port,baud_rate,incoming_data_send, outgoing_data_rcv)) #multiprocessing.Process(target=main_serial_process, args=(port,baud_rate,incoming_data_send, force_data_rcv)) #multiprocessing.Process(target=main_serial_process, args=(port,baud_rate,incoming_data_send, outgoing_data_rcv))
    p_serial.start()

    # assist process 
    #p_assist = multiprocessing.Process(target=main_assist_process, args=(force_data_send, incoming_data_rcv))
    #p_assist.start()

    # main exo control process 
    p_control = multiprocessing.Process(target=main_control_process, args=(incoming_data_rcv, outgoing_data_send)) #multiprocessing.Process(target=main_control_process, args=(force_data_rcv, outgoing_data_send)) #multiprocessing.Process(target=main_control_process, args=(incoming_data_rcv, outgoing_data_send))
    p_control.start()

    p_serial.join()
    print("Serial process finished")

    p_control.join()
    print("Control process finished")

    #p_assist.join()
    #print("Assist process finished")
    