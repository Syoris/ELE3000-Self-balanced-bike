import serial
import numpy as np
import os
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import time

serPort = "COM22"

class SerialDataClass:
    def __init__(self):
        self.to_connect = True
        self.data_name = ""
        self.file_path = os.path.join("Data", self.data_name)
        self.ser = ''

        self.Kp = 0.0672
        self.Ki = 0.71
        self.Kd = 0.0034

        self.data = {   "Target speed": [], 
                        "Current speed": [], 
                        "Command": [], 
                        "Angle": [],
                        "Time": [],
                        "Kp":self.Kp,
                        "Ki":self.Ki,
                        "Kd":self.Kd,
                        "Step amplitude": 0}

        self.data_interval = 0.005    # 1ms

        self.targetSpeed = 1500


        self.commands = {
            "run" : self.readSerialData,
            "show" : self.showData,
            "load" : self.loadData,
            "setSpeed" : self.setTargetSpeed,
            "setKp" : self.setKp,
            "setKi" : self.setKi,
            "setKd" : self.setKd,
            "step" : self.step,
            "idParam": self.parameterIdentification,
            "help" : self.print_help,
            "exit" : self.stop
        }

    def showData(self):
        # To show the data
        x = self.data["Time"]

        # Plot
        plt.plot(x, self.data["Target speed"], 'r', label="Target speed [deg/s]")
        plt.plot(x, self.data["Current speed"], 'b', label="Current speed [deg/s]" )
        plt.plot(x, self.data["Command"], 'k', label="Command [V]")
        plt.plot(x, self.data["Angle"], 'g', label="Angle [deg]")


        plt.title(self.data_name)
        plt.xlabel('Temps (sec)')
        plt.legend()
        plt.show()

    # Read Serial data
    def readSerialData(self):
        self.data["Target speed"] = []
        self.data["Current speed"] = []
        self.data["Command"] = []
        self.data["Angle"] = []
        self.data["Time"] = []

        input_type = input("Input type(step/ramp): ")
        if input_type == "step":
            goalSpeed = input("Target speed: ")
            self.setTargetSpeed(goalSpeed)
            startCom = "#On "
        elif input_type == "ramp":
            goalAccel = input("Target accel: ")
            startCom = "#accel " + goalAccel + " "
        else:
            print("Invalid type")
            startCom = " "
        
        endCom = "#Off "

        print("Reading data ...")
        self.ser.write(startCom.encode())
        self.ser.flushInput()
        self.ser.flushOutput()
        try:
            #Format des données: #target_speed, current_speed, speedCommand
            while True:
                serialData = self.ser.readline().decode('utf-8')
                if serialData.startswith('#'):
                    # print("Received data: ", serialData)
                    ser_data = serialData.replace(' ', '')
                    ser_data = ser_data.split(',')
                    ser_data[0] = ser_data[0][1:-1]
                    ser_data[-1] = ser_data[-1].rstrip(', \n\r')
                    
                    self.data["Target speed"].append(float(ser_data[0]))    # Target Speed
                    self.data["Current speed"].append(float(ser_data[1]))    # Current Speed
                    self.data["Command"].append(float(ser_data[2])*6/256)  # Command
                    self.data["Angle"].append(float(ser_data[3]))  # Angle
                    self.data["Time"].append(float(ser_data[4])) #Time

        except KeyboardInterrupt:
            print("Interrupted")
            self.ser.write(endCom.encode())
            self.ser.flushInput()
            serialData = self.ser.readline().decode('utf-8')
            while(not (serialData.startswith('!'))):
                serialData = self.ser.readline().decode('utf-8')

            ser_data = serialData.replace(' ', '')
            ser_data = ser_data.split(',')
            ser_data[0] = ser_data[0][1:]
            ser_data[-1] = ser_data[-1].rstrip(', \n\r')
            print("Kp:", ser_data[0], "\tKi:", ser_data[1], "\tKd:", ser_data[2] )

            self.ser.flushInput()

            # Correct time
            sub_val = self.data["Time"][0]
            self.data["Time"] = [(x - sub_val)/1000 for x in self.data["Time"]]

            self.saveData()

    def saveData(self):
        ans = input("Save data? (y/n) : ")
        if ans is "y":
            self.data_name = input("Enter save name: ")
            self.file_path = os.path.join("Data", self.data_name)
            np.save(self.file_path, self.data)
            print("Data saved")
        else:
            pass

    def getCommand(self):
        command = input("\nEnter command : ")
        command = command.split(' ')
        func_name = command[0]
        ok_function = True

        try:
            arg = command[1]
        except:
            arg = ''

        try:
            func = self.commands[func_name]
        except:
            ok_function = False
            print("Invalid function")

        if ok_function:
            if arg != '':
                func(arg)
            else:
                func()
    
    # Connect to Serial Port
    def connect(self):
        port_open = False
        print("Connecting...")
        while not port_open:
            try:
                self.ser = serial.Serial(serPort, timeout=None, baudrate=115000, xonxoff=False, rtscts=False, dsrdtr=False)
                self.ser.flushInput()
                self.ser.flushOutput()
                port_open = True
                print("Connected to ", self.ser.name)
            except:
                print("Connection failed")
                raise
        
        self.setTargetSpeed(self.targetSpeed)
        self.setKp(self.Kp)
        self.setKi(self.Ki)
        self.setKd(self.Kd)
    
    def stop(self):
        os._exit(0)

    def print_help(self):
        print("Possible commands :")
        for each_key in self.commands.keys():
            print('\t', each_key)

    def loadData(self, save_name):
        self.save_name = save_name

        print("Save number :", self.save_name)

        self.file_path = os.path.join("Data", self.save_name + ".npy")

        if os.path.isfile(self.file_path):
            self.data = np.load(self.file_path, allow_pickle=True).item()
            print("Data loaded")
        else:
            print("No data found")

    def setTargetSpeed(self, speed):
        self.targetSpeed = speed
        commande = "#setSpeed " + str(speed) + " "
        self.ser.write(commande.encode())
        serialData1 = self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')
    
    def setKp(self, Kp):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.Kp = Kp
        commande = "#setKp " + str(Kp) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')
    
    def setKi(self, Ki):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.Ki = Ki
        commande = "#setKi " + str(Ki) + " "
        self.ser.write(commande.encode())
        serialData1 = self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def setKd(self, Kd):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.Kd = Kd
        commande = "#setKd " + str(Kd) + " "
        self.ser.write(commande.encode())
        serialData1 = self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def step(self, step_amplitude):
        self.data["Step amplitude"] = step_amplitude

        self.data["Target speed"] = []
        self.data["Current speed"] = []
        self.data["Command"] = []
        self.data["Angle"] = []
        self.data["Time"] = []

        print("Reading data ...")
        startCom = "#step " + str(step_amplitude) + " "
        self.ser.write(startCom.encode())
        serialData = self.ser.readline().decode('utf-8')
        serialData = self.ser.readline().decode('utf-8')

        self.ser.flushInput()
        self.ser.flushOutput()
        #Format des données: #target_speed, current_speed, speedCommand
        while True:
            serialData = self.ser.readline().decode('utf-8')
            # if time.time() > timeout:
            #     break
            if serialData.startswith('#'):
                print("Received data: ", serialData)
                ser_data = serialData.replace(' ', '')
                ser_data = ser_data.split(',')
                ser_data[0] = ser_data[0][1:-1]
                ser_data[-1] = ser_data[-1].rstrip(', \n\r')
                
                self.data["Target speed"].append(float(ser_data[0]))    # Target Speed
                self.data["Current speed"].append(float(ser_data[1]))    # Current Speed
                self.data["Command"].append(float(ser_data[2])*6/256)  # Command
                self.data["Angle"].append(float(ser_data[3]))  # Angle
                self.data["Time"].append(float(ser_data[4]))  # Angle

            elif serialData.startswith('*'):
                break
        print("Timeout")
        self.ser.flushInput()

        sub_val = self.data["Time"][0]
        self.data["Time"] = [(x - sub_val)/1000 for x in self.data["Time"]]

        # Calculate motor parameters
        ans = input("Motor parameters identification? (y/n) : ")
        if ans is "y":
            self.parameterIdentification()
        else:
            pass
    
        # Save data
        self.saveData()

    def parameterIdentification(self):
        step_amplitude = self.data["Step amplitude"]
        start_data = 50

        y = self.data["Angle"]
        x = self.data["Time"]

        y = y[start_data:-1]
        x = x[start_data:-1]

        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]

        print("m: ", m)
        print("b: ", b)

        K = m/float(step_amplitude)
        tau = -b/m
        print("\nParamètres du moteur")
        print("\tStep: ", step_amplitude)
        print("\tK: ", K)
        print("\tTau: ", tau)

        y_droite = [j*m+b for j in x]

        show_res = True

        if show_res:
            plt.plot(x, y, 'k', label='Original data')
            plt.plot(x, y_droite, 'r', label='Fitted line')
            plt.legend()
            plt.show()



def main():
    serData = SerialDataClass()
    if serData.to_connect:
        serData.connect()

    while True:
        try:
            serData.getCommand()
        except KeyboardInterrupt:
            print("End of program")

    os._exit(0)

if __name__ == "__main__":
    main()