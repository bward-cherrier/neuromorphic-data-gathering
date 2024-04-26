import time, socket, pickle
import numpy as np

from vsp.processor import Processor
from vsp.processor import AsyncProcessor
from vsp.utils import compose

# Imports required for serial comms with sensor
from math import *
import serial
import minimalmodbus as mm
import io
import libscrc

# Class to interface with FT sensor through ethernet connection to UR5 controller
# Currently issues with RTDE connection to robot being dropped - change timeout to fix?

class FTSensor():

    def __init__(self, host= '192.11.72.10', port = 63351):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data = []
        self.connect()

    def reset(self):
        self.data = []

    def connect(self):
        try:
            print("Connecting FT sensor to " + self.host) 
            self.socket.connect((self.host, self.port))
            print("Connection successful")
        except:
            print ("No connection")

    def read(self, outfile=None): 
        ft_data = self.socket.recv(67).decode()
        # self.data.append([time.time(), ft_data])
        return [time.time(), [float(x) for x in ft_data[1:-1].split(', ')]]

    def close(self):
        pass


# Class to interface with the sensor using a USB serial connection
# This class is to be used if interfacing with the ft sensor directly
#   - NOT VIA THE UR-5/10 
# Class based on code by Benoit CASTETS - PROVIDE URL HERE
# TODO: Use threading to allow for continuous stream of data
class FTSensorSerial():
    def __init__(self, port="/dev/ttyUSB0", zero_ref=[0, 0, 0, 0, 0, 0]):
        # Set port accordingly when creating class
        # Port defaults to Linux default port 
        # Default port for Windows: "COM3"
        # Default port for Linux: "/dev/ttyUSB0"
        self.PORTNAME = port

        # Communication setup
        # These values are all defaults
        self.BAUDRATE = 19200
        self.BYTESIZE = 8
        self.PARITY = "N"
        self.STOPBITS = 1
        self.TIMEOUT = 1
        self.SLAVEADDRESS = 9
        self.STARTBYTES = bytes([0x20, 0x4e])

        # Zero reference of sensor - needs resetting each startup
        self.zero_ref = zero_ref

        # Stop any streaming data on port
        self.__stop_streaming()
        
    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()

    def close(self):
        pass

    def __forceFromSerialMessage(self, serialMessage):
        """Return a list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] correcponding to the dataArray
        Parameters
        ----------
        serialMessage:
        bytearray which contents the serial message send by the FT300.
        [0x20,0x4e,LSBdata1,MSBdata2,...,LSBdata6,MSBdata6,crc1,crc2]
        Check FT300 manual for details.
        zeroRef:
        list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] use the set the zero reference of the sensor.
        
        Return
        ------
        forceTorque:
        list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] correcponding to the dataArray
        """
        #Initialize variable
        forceTorque = [0, 0, 0, 0, 0, 0]

        #converte bytearray values to integer. Apply the zero offset and round at 2 decimals
        forceTorque[0] = round(int.from_bytes(
            serialMessage[2:4], byteorder='little', signed=True)/100-self.zero_ref[0], 2)
        forceTorque[1] = round(int.from_bytes(
            serialMessage[4:6], byteorder='little', signed=True)/100-self.zero_ref[1], 2)
        forceTorque[2] = round(int.from_bytes(
            serialMessage[6:8], byteorder='little', signed=True)/100-self.zero_ref[2], 2)
        forceTorque[3] = round(int.from_bytes(
            serialMessage[8:10], byteorder='little', signed=True)/1000-self.zero_ref[3], 2)
        forceTorque[4] = round(int.from_bytes(
            serialMessage[10:12], byteorder='little', signed=True)/1000-self.zero_ref[4], 2)
        forceTorque[5] = round(int.from_bytes(
            serialMessage[12:14], byteorder='little', signed=True)/1000-self.zero_ref[5], 2)

        return forceTorque

    def __crcCheck(self, serialMessage):
        """Check if the serial message have a valid CRC.
        
        Parameters
        -----------
        serialMessage:
        bytearray which contents the serial message send by the FT300.
        [0x20,0x4e,LSBdata1,MSBdata2,...,LSBdata6,MSBdata6,crc1,crc2]
        Check FT300 manual for details.
        
        Return
        ------
        checkResult:
        bool, return True if the message have a valid CRC and False if not.
        """
        checkResult = False

        #CRC from serial message
        crc = int.from_bytes(
            serialMessage[14:16], byteorder='little', signed=False)
        #calculated CRC
        crcCalc = libscrc.modbus(serialMessage[0:14])

        if crc == crcCalc:
            checkResult = True

        return checkResult

    # Function sends stop characters to sensor in order to stop data stream
    def __stop_streaming(self):
        #To stop the data stream, communication must be interrupted by sending a series of 0xff characters to the Sensor. Sending for about
        #0.5s (50 times)will ensure that the Sensor stops the stream.
        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)
        packet = bytearray()
        sendCount = 0
        while sendCount < 50:
            packet.append(0xff)
            sendCount = sendCount+1
        ser.write(packet)
        ser.close()

    # Calibration function to set reference values for ft
    def set_zero_ref(self):
        ####################
        #Setup minimalmodbus
        ####################
        mm.BAUDRATE = self.BAUDRATE
        mm.BYTESIZE = self.BYTESIZE
        mm.PARITY = self.PARITY
        mm.STOPBITS = self.STOPBITS
        mm.TIMEOUT = self.TIMEOUT

        #Create FT300 object
        ft300 = mm.Instrument(self.PORTNAME, slaveaddress=self.SLAVEADDRESS)
        ft300.close_port_after_each_call = True

        #Uncomment to see binary messages for debug
        #ft300.debug=True
        #ft300.mode=mm.MODE_RTU

        #Write 0x0200 in 410 register to start streaming
        ###############################################
        registers = ft300.write_register(410, 0x0200)

        del ft300

        ############################
        #Open serial connection
        ############################

        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)

        ############################
        #Initialize stream reading
        ############################
        #Read serial buffer until founding the bytes [0x20,0x4e]
        #First serial reading.
        #This message in uncomplete in most cases so it is ignored.
        data = ser.read_until(self.STARTBYTES)

        # Take second reading as first is often incomplete
        data = ser.read_until(self.STARTBYTES)
        #convert from byte to bytearray
        data_array = bytearray(data)
        #Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
        data_array = self.STARTBYTES + data_array[:-2]

        if self.__crcCheck(data_array) is False:
            raise Exception(
                "CRC ERROR: Serial message and the CRC does not match")

        self.zero_ref = self.__forceFromSerialMessage(data_array)

        self.__stop_streaming()

    # Function to get single ft reading from sensor based on previously set zero reference
    # NOTE: MUST SET A ZERO REFERENCE USING "set_zero_ref" FUNCTION BEFORE USE
    # NOTE: CURRENTLY ERRORS OUT IF REQUESTS FOR FT DATA COME IN TOO QUICKLY 
    #           - PERHAPS ADD A SMALL DELAY BETWEEN FUNCTION CALLS
    def read(self):
        ####################
        #Setup minimalmodbus
        ####################
        mm.BAUDRATE = self.BAUDRATE
        mm.BYTESIZE = self.BYTESIZE
        mm.PARITY = self.PARITY
        mm.STOPBITS = self.STOPBITS
        mm.TIMEOUT = self.TIMEOUT

        #Create FT300 object
        ft300 = mm.Instrument(self.PORTNAME, slaveaddress=self.SLAVEADDRESS)
        ft300.close_port_after_each_call = True

        #Uncomment to see binary messages for debug
        #ft300.debug=True
        #ft300.mode=mm.MODE_RTU

        #Write 0x0200 in 410 register to start streaming
        ###############################################
        registers = ft300.write_register(410, 0x0200)

        del ft300

        ############################
        #Open serial connection
        ############################

        ser = serial.Serial(port=self.PORTNAME, baudrate=self.BAUDRATE, bytesize=self.BYTESIZE,
                            parity=self.PARITY, stopbits=self.STOPBITS, timeout=self.TIMEOUT)

        timestamp = time.time()
        ############################
        #Initialize stream reading
        ############################
        #Read serial buffer until founding the bytes [0x20,0x4e]
        #First serial reading.
        #This message in uncomplete in most cases so it is ignored.
        data = ser.read_until(self.STARTBYTES)
        
        # Take second reading as first is often incomplete
        data = ser.read_until(self.STARTBYTES)
        #convert from byte to bytearray
        data_array = bytearray(data)
        #Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
        data_array = self.STARTBYTES + data_array[:-2]

        # Convert byte array into a list of ft values
        ft_data = self.__forceFromSerialMessage(data_array)
        ft_data.append(timestamp)
        
        # Check data is valid
        if self.__crcCheck(data_array) is False:
            raise Exception(
                "CRC ERROR: Serial message and the CRC does not match")

        # Stop streaming so next test can start it again
        self.__stop_streaming()
        
        time.sleep(0.02)

        # Return list of ft readings
        return ft_data


class FTSensorProcessor(Processor):

    def __init__(self, ft_sensor, pipeline=[], view=None, display=None, writer=None):
        self.ft_sensor = ft_sensor
        self.pipeline = pipeline
        self.view = view
        self.display = display
        self.writer = writer

    def process(self, num_frames, outfile=None):
         # initialization
        if len(self.pipeline) > 0:
            pipeline_func = compose(*self.pipeline[::-1])

        if self.display:
            if len(self.pipeline) > 0:
                display_func = compose(self.display.write, self.view.draw)
            else:
                display_func = self.display.write
            self.display.open()

        # if self.writer and outfile:
        #     self.writer.filename = outfile
        #     self.writer.open()

        # run pipeline
        results = []
        self._cancel = False
        for i in range(num_frames):
            if self._cancel:
                break

            inp = self.ft_sensor.read()

            if len(self.pipeline) > 0:
                out = pipeline_func(inp)
            else:
                out = inp

            if self.display:
                if len(self.pipeline) > 0:
                    display_func(inp, out)
                else:
                    display_func(inp)

            # if self.writer and outfile:
            #     self.writer.write(inp)

            results.append(out)

        # termination
        if self.display:
            self.display.close()
        if self.writer and outfile:
            with open(outfile, 'wb') as t:
                pickle.dump(results, t)
            # self.writer.close()
        

        return np.array(results)

    def cancel(self):
        self._cancel = True

    def close(self):
        self.ft_sensor.close()

###############
# Main program
###############
if __name__ == "__main__":
 
    ft = FTSensorSerial()
    ft.set_zero_ref()

    ft_processor = AsyncProcessor(FTSensorProcessor(ft_sensor = ft, writer = True))
    ft_processor.async_process(num_frames = 10000, outfile = "test.pkl")
    
    time.sleep(5)
    
    # Stop FT sensor
    ft_processor.async_cancel()
    _ = ft_processor.async_result()
    
    ft_processor.close()
    
    ft.close()
    
    with open('test.pkl', 'rb') as file:
        data =  pickle.load(file)

    # print(data)
    while True:
        print(ft.read())
        time.sleep(0.1)
