import serial
import numpy as np
import os
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import time
import scipy.io

serPort = "COM3"

# p = -25
# KpM = 0.007052  
# KiM = 0.195357
# KdM = 0

# p = -40
KpM = 0.011740  
KiM = 0.500114
KdM = 0 


KpV = -2650
KiV = -0
KdV = -50

# Essai 1
# Kpv = -2175
# Kdv = -432

class SerialDataClass:
    def __init__(self):
        self.to_connect = True
        self.data_name = ""
        self.file_path = os.path.join("Data", self.data_name)
        self.ser = ''

        # self.KpM = 0.0672
        # self.KiM = 0.71
        # self.KdM = 0.0034
        self.KpM = KpM
        self.KiM = KiM
        self.KdM = KdM


        self.KpV = KpV
        self.KiV = KiV
        self.KdV = KdV

        self.data = {   "BikeAngle": [],
                        "TargetSpeed": [], 
                        "CurrentSpeedRaw": [],
                        "CurrentSpeed": [], 
                        "Command": [], 
                        "MotorAngle": [],
                        "AngularVelRaw": [],
                        "AngularVel": [],
                        "TargetAccel": [],
                        "Time": [],
                        "KpM":self.KpM,
                        "KiM":self.KiM,
                        "KdM":self.KdM,
                        "KpV":self.KpV,
                        "KiV":self.KiV,
                        "KdV":self.KdV,
                        "StepAmplitude": 0}

        self.targetSpeed = 0

        self.commands = {
            "run" : self.readSerialData,
            "r" : self.readSerialData,
            "show" : self.showData,
            "load" : self.loadData,
            "save" : self.saveData,
            "setSpeed" : self.setTargetSpeed,
            "setKpM" : self.setKpM,
            "setKiM" : self.setKiM,
            "setKdM" : self.setKdM,
            "p" : self.setKpV,
            "i" : self.setKiV,
            "d" : self.setKdV,
            "step" : self.step,
            "idParam": self.parameterIdentification,
            "angle": self.checkAngle,
            "help" : self.print_help,
            "exit" : self.stop
        }

    def showData(self):
        showRaw = True

        # To show the data
        x = self.data["Time"]

        # Plot
        fig, axs = plt.subplots(2, sharex=True)
        fig.suptitle(self.data_name)
        axY = axs[0].twinx()
        axs[0].plot(x, self.data["TargetSpeed"], 'r', label="Target speed [deg/s]")
        axs[0].plot(x, self.data["CurrentSpeed"], 'b', label="Current speed [deg/s]" )
        # axY.plot(x, self.data["TargetAccel"], 'c', label="Target Accel [deg/s**2]" )
        if(showRaw) : axs[0].plot(x, self.data["CurrentSpeedRaw"], 'y', label="Raw speed [deg/s]" )
        # axs[0].plot(x, self.data["MotorAngle"], 'g', label="Angle [deg]")
        axs[0].legend()
        axs[0].grid(True, which='both')
        axs[0].axhline(y=0, color='k', linestyle='dashed')
        # axY.axhline(y=0, color='c', linestyle = 'dashed')

        axs[0].axvline(x=0, color='k')
        axs[0].set_xlim(xmin=0)

        axY = axs[1].twinx()
        axs[1].plot(x, self.data["BikeAngle"], 'g', label="Bike angle [deg]")
        # axs[1].plot(x, self.data["Command"], 'k', label="Commande [V]")
        axY.plot(x, self.data["AngularVel"], 'c', label="Angular Velocity [deg/sec]")
        if(showRaw) : axY.plot(x, self.data["AngularVelRaw"], 'y', label="Raw Angular Velocity [deg/sec]")

        
        axY.set_ylabel("Angular Velocity [deg/sec]", color='c')
        axs[1].set_ylabel("Angular Position [deg]", color='g')

        axY.tick_params(axis='y', labelcolor = 'c')
        axs[1].tick_params(axis='y', labelcolor = 'g')

        axY.grid(True, which='both')
        axs[1].grid(True, which='both')

        axs[1].axhline(y=0, color='g', linestyle = 'dashed')
        axY.axhline(y=0, color='c', linestyle = 'dashed')
        axs[1].axvline(x=0, color='k')
        axs[1].set_xlim(xmin=0)

        fig.tight_layout()
        plt.xlabel('Temps (sec)')
        plt.show()

    # Read Serial data
    def readSerialData(self, in_type=""):
        self.data["BikeAngle"] = []
        self.data["TargetSpeed"] = []
        self.data["CurrentSpeedRaw"] = []
        self.data["CurrentSpeed"] = []
        self.data["Command"] = []
        self.data["MotorAngle"] = []
        self.data["AngularVelRaw"] = []
        self.data["AngularVel"] = []
        self.data["TargetAccel"] = []
        self.data["Time"] = []


        if in_type == "": input_type = input("Input type(stabilize/speed/accel): ")
        elif in_type=="s": input_type = "stabilize"
        elif in_type=="accel": input_type = "accel"
        elif in_type=="speed": input_type = "speed"
        else: 
            print("Invalid type")
            return
            

        

        if input_type == "speed":
            goalSpeed = input("Target speed: ")
            self.setTargetSpeed(goalSpeed)
            startCom = "#On "
        elif input_type == "accel":
            goalAccel = input("Target accel: ")
            startCom = "#ramp " + goalAccel + " "
        elif input_type == "stabilize":
            startCom = "#stabilize "
        else:
            print("Invalid type")
            return
        
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
                    
                    self.data["BikeAngle"].append(float(ser_data[0])*180/np.pi) # Angle du vélo
                    self.data["TargetSpeed"].append(float(ser_data[1]))    # Target Speed
                    self.data["CurrentSpeedRaw"].append(float(ser_data[2]))    # Current Speed
                    self.data["CurrentSpeed"].append(float(ser_data[3]))    # Current Speed
                    self.data["Command"].append(float(ser_data[4]))  # Command
                    self.data["MotorAngle"].append(float(ser_data[5]))  # Angle du moteur
                    self.data["AngularVelRaw"].append(float(ser_data[6])*180/np.pi)  # Vitesse angulaire
                    self.data["AngularVel"].append(float(ser_data[7])*180/np.pi)  # Vitesse angulaire filtré
                    self.data["TargetAccel"].append(float(ser_data[8]))  # Vitesse angulaire filtré
                    self.data["Time"].append(float(ser_data[9])) #Time
                
                elif serialData.startswith('*'):
                    print("Timeout")
                    break

        except KeyboardInterrupt:
            print("Interrupted")
            self.ser.write(endCom.encode())

        # self.ser.flushInput()
        # serialData = self.ser.readline().decode('utf-8')

        while(not (serialData.startswith('!'))):
            serialData = self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        
        ser_data = serialData.replace(' ', '')
        ser_data = ser_data.split(',')
        ser_data[0] = ser_data[0][1:]
        ser_data[-1] = ser_data[-1].rstrip(', \n\r')
        print("Gains du moteur")
        print("Kp:", ser_data[0], "\tKi:", ser_data[1], "\tKd:", ser_data[2] )

        ser_data = serialData2.replace(' ', '')
        ser_data = ser_data.split(',')
        ser_data[0] = ser_data[0][1:]
        ser_data[-1] = ser_data[-1].rstrip(', \n\r')
        print("Gains du vélo")
        print("Kp:", ser_data[0], "\tKi:", ser_data[1], "\tKd:", ser_data[2] )

        self.ser.flushInput()

        # Correct time
        sub_val = self.data["Time"][0]
        self.data["Time"] = [(x - sub_val)/1000 for x in self.data["Time"]]

        print("Angle inital: ", self.data["BikeAngle"][0])
        if(self.data["BikeAngle"][0] > 7 or self.data["BikeAngle"][0] < 6): print("!!!ALERTE!!!")
        # self.saveData()

    def saveData(self):
        self.data_name = input("Enter save name: ")
        ans = input("Save format? (p/m/b) : ")

        if ans is "p" or ans is "b":
            self.file_path = os.path.join("Data", self.data_name)
            np.save(self.file_path, self.data)
            print("Data saved for Python")
        
        if ans is "m" or ans is "b":
            self.file_path = os.path.join(os.path.dirname(os.getcwd()), "Simulations")
            self.file_path = os.path.join(self.file_path, "PythonData")
            self.file_path = os.path.join(self.file_path, self.data_name + '.mat')
            scipy.io.savemat(self.file_path, self.data)
            print("Data saved for Matlab")
     

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
            try:
                pass
            except TypeError:
                print("Invalid parameters")
                return
    
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
        self.setKpM(self.KpM)
        self.setKiM(self.KiM)
        self.setKdM(self.KdM)
        self.setKpV(self.KpV)
        self.setKiV(self.KiV)
        self.setKdV(self.KdV)
    
    def stop(self):
        os._exit(0)

    def print_help(self):
        print("Possible commands :")
        for each_key in self.commands.keys():
            print('\t', each_key)

    def checkAngle(self):
        startCom = "#checkAngle "
        
        endCom = "#Off "

        print("Printing angle...")
        self.ser.write(startCom.encode())
        self.ser.flushInput()
        self.ser.flushOutput()
        try:
            while True:
                serialData = self.ser.readline().decode('utf-8')
                if serialData.startswith('#'):
                    ser_data = serialData.replace(' ', '')
                    ser_data = ser_data[1:-1]
                    ser_data = float(ser_data.rstrip(', \n\r'))
                    print("Angle: ", ser_data*180/np.pi)
                    
        except KeyboardInterrupt:
            print("\nInterrupted")
            self.ser.write(endCom.encode())

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
    
    def setKpM(self, Kp):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KpM = Kp
        commande = "#setKpM " + str(Kp) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')
    
    def setKiM(self, Ki):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KiM = Ki
        commande = "#setKiM " + str(Ki) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def setKdM(self, Kd):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KdM = Kd
        commande = "#setKdM " + str(Kd) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def setKpV(self, Kp):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KpV = Kp
        commande = "#setKpV " + str(Kp) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')
    
    def setKiV(self, Ki):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KiV = Ki
        commande = "#setKiV " + str(Ki) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def setKdV(self, Kd):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.KdV = Kd
        commande = "#setKdV " + str(Kd) + " "
        self.ser.write(commande.encode())
        self.ser.readline().decode('utf-8')
        serialData2 = self.ser.readline().decode('utf-8')
        print(serialData2, end='')

    def step(self, step_amplitude):
        self.data["StepAmplitude"] = step_amplitude

        self.data["TargetSpeed"] = []
        self.data["CurrentSpeed"] = []
        self.data["Command"] = []
        self.data["Angle"] = []
        self.data["Time"] = []

        print("Reading data ...")
        startCom = "#step " + str(step_amplitude) + " "
        self.ser.write(startCom.encode())
        serialData = self.ser.readline().decode('utf-8')
        serialData = self.ser.readline().decode('utf-8')

        # self.ser.flushInput()
        # self.ser.flushOutput()
        #Format des données: #target_speed, current_speed, speedCommand

        while True:
            serialData = self.ser.readline().decode('utf-8')
            
            if serialData.startswith('#'):
                # print("Received data: ", serialData)
                ser_data = serialData.replace(' ', '')
                ser_data = ser_data.split(',')
                ser_data[0] = ser_data[0][1:-1]
                ser_data[-1] = ser_data[-1].rstrip(', \n\r')
                
                self.data["BikeAngle"].append(float(ser_data[0])) 
                self.data["TargetSpeed"].append(float(ser_data[1]))    # Target Speed
                self.data["CurrentSpeed"].append(float(ser_data[2]))    # Current Speed
                self.data["Command"].append(float(ser_data[3])*6/256)  # Command
                self.data["MotorAngle"].append(float(ser_data[4]))  # Angle
                self.data["Time"].append(float(ser_data[5])) #Time

            elif serialData.startswith('*'):
                break

        print("Timeout")
        self.ser.flushInput()

        sub_val = self.data["Time"][0]
        self.data["Time"] = [(x - sub_val)/1000 for x in self.data["Time"]]

        # Calculate motor parameters
        self.ser.flushInput()
        ans = input("Motor parameters identification? (y/n) : ")
        if ans is "y":
            self.parameterIdentification()
        else:
            pass


    def parameterIdentification(self):
        step_amplitude = self.data["StepAmplitude"]
        start_data = 500

        y = self.data["MotorAngle"]
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
            plt.title("Parameter identification")


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