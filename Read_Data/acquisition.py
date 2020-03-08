import serial
import numpy as np
import os
import matplotlib.pyplot as plt
import time

serPort = "COM22"

class SerialDataClass:
    def __init__(self):
        self.to_connect = True
        self.save_number = 1
        self.file_path = os.path.join("Data", "Data_" + str(self.save_number))
        self.ser = ''
        self.data = {"Target speed": [], "Current speed": [], "Command": []}
        self.data_interval = 0.1    # 200ms

        self.targetSpeed = 1000
        self.Kp = 0.5
        self.Ki = 0
        self.Kd = 0

        self.commands = {
            "getData" : self.readSerialData,
            "showData" : self.showData,
            "loadData" : self.loadData,
            "setSpeed" : self.setTargetSpeed,
            "setKp" : self.setKp,
            "setKi" : self.setKi,
            "setKd" : self.setKd,
            "help" : self.print_help,
            "exit" : self.stop
        }

    def showData(self):
        # To show the data
        x = np.linspace(0, len(self.data["Target speed"]) - 1, num=len(self.data["Target speed"]))
        x = x*self.data_interval

        # Plot
        plt.plot(x, self.data["Target speed"], 'r', label="Target speed")
        plt.plot(x, self.data["Current speed"], 'b', label="Current speed" )
        plt.plot(x, self.data["Command"], 'k', label="Command")

        plt.title("Data " + str(self.save_number))
        plt.xlabel('Temps (sec)')
        plt.legend()
        plt.show()

    # Read Serial data
    def readSerialData(self):
        self.data = {"Target speed": [], "Current speed": [], "Command": []}
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
                    self.data["Command"].append(float(ser_data[2]))  # Command

        except KeyboardInterrupt:
            print("Interrupted")
            self.ser.flushInput()
            self.ser.write(endCom.encode())
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
            np.save(self.file_path, self.data)
            print("Data saved")
        else:
            pass

    def getCommand(self):
        command = input("\nEnter command : ")
        command = command.split(' ')
        func_name = command[0]
        try:
            arg = command[1]
        except:
            arg = ''

        func = self.commands[func_name]

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

    def loadData(self, save_number):
        self.save_number = save_number

        print("Save number :", self.save_number)

        self.file_path = os.path.join("Data", "Data_" + self.save_number+'.npy')

        if os.path.isfile(self.file_path):
            self.data = np.load(self.file_path, allow_pickle=True).item()
            print("Data loaded")
        else:
            print("No data found, creating new set")

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
        serialData1 = self.ser.readline().decode('utf-8')
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