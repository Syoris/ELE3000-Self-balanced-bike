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
        self.to_connect = False
        self.data_name = ""
        self.file_path = os.path.join("Data", self.data_name)
        self.ser = ''

        self.Kp = 0.5
        self.Ki = 0
        self.Kd = 0

        self.data = {   "Target speed": [], 
                        "Current speed": [], 
                        "Command": [], 
                        "Angle": [], 
                        "Kp":self.Kp,
                        "Ki":self.Ki,
                        "Kd":self.Kd}

        self.data_interval = 0.001    # 1ms

        self.targetSpeed = 1000


        self.commands = {
            "getData" : self.readSerialData,
            "showData" : self.showData,
            "loadData" : self.loadData,
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
        x = np.linspace(0, len(self.data["Target speed"]) - 1, num=len(self.data["Target speed"]))
        x = x*self.data_interval

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
        
        print("Reading data ...")
        startCom = "#On "
        endCom = "#Off "
        self.ser.write(startCom.encode())

        self.ser.flushInput()
        self.ser.flushOutput()
        try:
            #Format des données: #target_speed, current_speed, speedCommand
            while True:
                serialData = self.ser.readline().decode('utf-8')
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


        except KeyboardInterrupt:
            print("Interrupted")
            self.ser.write(endCom.encode())
            self.ser.flushInput()
            self.ser.readline().decode('utf-8')
            self.ser.readline().decode('utf-8')
            serialData = self.ser.readline().decode('utf-8')
            ser_data = serialData.replace(' ', '')
            ser_data = ser_data.split(',')
            ser_data[0] = ser_data[0][1:-1]
            ser_data[-1] = ser_data[-1].rstrip(', \n\r')
            print("Kp:", ser_data[0], "\tKi:", ser_data[1], "\tKd:", ser_data[2] )

            self.ser.flushInput()
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
        self.data["Target speed"] = []
        self.data["Current speed"] = []
        self.data["Command"] = []
        self.data["Angle"] = []
        
        print("Reading data ...")
        startCom = "#step " + str(step_amplitude) + " "
        self.ser.write(startCom.encode())
        serialData = self.ser.readline().decode('utf-8')
        serialData = self.ser.readline().decode('utf-8')

        self.ser.flushInput()
        self.ser.flushOutput()
        timeout = time.time() + 3
        #Format des données: #target_speed, current_speed, speedCommand
        while True:
            serialData = self.ser.readline().decode('utf-8')
            if time.time() > timeout:
                break
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


        print("Timeout")
        self.ser.flushInput()
        self.saveData()
    
    def parameterIdentification(self, step_amplitude):
        start_time = 0
        n_data = 50
        start_data = int(start_time/self.data_interval)

        y = self.data["Angle"]
        x = np.linspace(0, len(self.data["Target speed"]) - 1, num=len(self.data["Target speed"]))
        x = x*self.data_interval

        y = y[start_data:start_data+n_data]
        x = x[start_data:start_data+n_data]

        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]

        print("m: ", m)
        print("b: ", b)

        K = m/float(step_amplitude)
        tau = -b/m
        print("\nParamètres du moteur")
        print("\tK: ", K)
        print("\tTau: ", tau)

        plt.plot(x, y, 'k', label='Original data')
        plt.plot(x, m*x + b, 'r', label='Fitted line')
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