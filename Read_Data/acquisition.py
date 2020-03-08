import serial
import numpy as np
import os
import matplotlib.pyplot as plt

serPort = "COM22"

class SerialDataClass:
    def __init__(self):
        self.to_connect = True
        self.save_number = 1
        self.file_path = os.path.join("Data", "Data_" + str(self.save_number))
        self.ser = ''
        self.data = {"Target speed": [], "Current speed": [], "Command": []}
        self.data_interval = 0.2    # 200ms

        self.commands = {
            "getData" : self.readSerialData,
            "showData" : self.showData,
            "loadData" : self.loadData,
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

        plt.title("Data " + self.save_number)
        plt.xlabel('Temps (sec)')
        plt.legend()
        plt.show()

    # Read Serial data
    def readSerialData(self):
        print("Reading data ...")
        self.ser.write(1)
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