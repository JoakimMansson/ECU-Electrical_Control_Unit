from tokenize import Double
from xml.etree.ElementInclude import DEFAULT_MAX_INCLUSION_DEPTH
import can as can
import serial
import time
import struct
import os
import RPi.GPIO as GPIO
import numpy as np
import serial
import schedule

import multiprocessing
import threading
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from CAN_receiver import ReaderCan




os.system("sudo ifconfig can1 up")
"""
0x00, 0x00, 0x7A, 0x44 = MAX RPM
0xD5, 0x78, 0x69, 0x3D = 0.057 CURRENT
"""

class ReceivePotential():

    def __init__(self) -> None:
        i2c = busio.I2C(board.SCL, board.SDA)

        # Create the ADC object using the I2C bus
        ads = ADS.ADS1015(i2c)

        # Create single-ended input on channel 0 and 1
        self.__brake_pot = AnalogIn(ads, ADS.P0)
        self.__gas_pot = AnalogIn(ads, ADS.P1)
    
    def get_brake_pot(self):
        return self.__brake_pot.value/100
    
    def get_gas_pot(self):
        return self.__gas_pot.value/100
    

class CanData():
    
    def __init__(self):
        self.DRIVE_ARR = [int("00",16), int("80",16), int("84",16), int("44",16), 0x00, 0x00, 0x00, 0x00]
        self.REVERSE_ARR = [int("00",16), int("80",16), int("84",16), int("c4",16), 0x00, 0x00, 0x00, 0x00]
        self.NEUTRAL_ARR = [int("00",16), int("80",16), int("84",16), int("44",16), 0x00, 0x00, 0x00, 0x00]
        self.BRAKE_ARR = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        self._CanData_observers = []

    def notify(self):
        for observer in self._CanData_observers:
            observer.fetch_data()
        
    def attach(self, observer) -> None:
        if observer not in self._CanData_observers:
            self._CanData_observers.append(observer)
        else:
            print("Observer already in list")

    def detach(self, observer) -> None:
        try:
            self._CanData_observers.remove(observer)
        except Exception as e:
            print("Observer not in list, can't remove")



class SendCan():

    def __init__(self, can_channel) -> None:
        self.can_port = can.interface.Bus(bustype = 'socketcan', channel = can_channel)
        self.BUS_VOLTAGE = [0,0,0,0,0,0,128,63]

    def send_bus_voltage(self) -> None:
        self.send_msg(0x502, self.BUS_VOLTAGE)


    def send_msg(self, can_id, data_arr) -> None:
        msg = can.Message(arbitration_id=can_id, data=data_arr, is_extended_id=False)
        self.can_port.send(msg)

# DECRYPT
class decrypt():

    #Used for sending
    def float_to_hex_msg(self,value: float) -> list:
        if value == 0:
            return [0,0,0,0]
    
        bits, = struct.unpack('!I', struct.pack('!f', value))
        bits = "{:032b}".format(bits)
        hex_value = str(hex(int(bits,2)))[2:]
    
        hex_arr = []
        for i in range(0,len(hex_value),2):
            hex_arr.append(int(hex_value[i:i+2],16))
      
        return hex_arr


# RAWCAN 
class RawCan():
    def __init__(self) -> None:
        self.potential = ReceivePotential()

        # Gas potentials
        self.raw_gas_potential = 0
        self.raw_max_gas_potential = 0
        self.raw_min_gas_potential = 99999

        # brake potentials
        self.raw_brake_potential = 0
        self.raw_max_brake_potential = 0
        self.raw_min_brake_potential = 99999

    # Call this function to update raw gas and brake potential
    def update_potentials(self):
        try:
            #Raw gas
            self.raw_gas_potential = self.potential.get_gas_pot()
            self.__is_new_max(True, self.raw_gas_potential)   
            
            #Raw brake
            self.raw_brake_potential = self.potential.get_brake_pot()
            self.__is_new_max(False, self.raw_brake_potential)
        
        except Exception as e:
            print("FAILED RAW_CAN: " + str(e))
        

    def __is_new_max(self, is_gas, potential):
        if is_gas:
            if potential > self.raw_max_gas_potential:
                self.raw_max_gas_potential = potential
            elif potential < self.raw_min_gas_potential:
                self.raw_min_gas_potential = potential
        else:
            if potential > self.raw_max_brake_potential:
                self.raw_max_brake_potential = potential
            elif potential != 0 and potential < self.raw_min_brake_potential:
                self.raw_min_brake_potential = potential
    
    # Return raw gas
    def get_raw_gas_potential(self):
        return self.raw_gas_potential
    
    # Return raw brake
    def get_raw_brake_potential(self):
        return self.raw_brake_potential

    def get_min_max_brake(self):
        return self.raw_max_brake_potential, self.raw_min_brake_potential

    def get_min_max_gas(self):
        return self.raw_max_gas_potential, self.raw_min_gas_potential

    def reset_min_max(self):
        # Gas potentials
        self.raw_max_gas_potential = 0
        self.raw_min_gas_potential = 99999

        # brake potentials
        self.raw_max_brake_potential = 0
        self.raw_min_brake_potential = 99999




######################## DRIVE ########################################
class DriveCan():
    
    def __init__(self, can_data: CanData) -> None:
        self.decrypt = decrypt()
        self.potential = ReceivePotential()
        self.can_data = can_data

        # Drive, reverse and neutral arrays
        self.DRIVE_ARR = can_data.DRIVE_ARR
        self.REVERSE_ARR = can_data.REVERSE_ARR
        self.NEUTRAL_ARR = can_data.NEUTRAL_ARR
        self.BRAKE_ARR = can_data.BRAKE_ARR

        # Drive state (0 = neutral, 1 = drive, 2 = reverse)
        self.drive_state = 0

        # Raw gas
        self.raw_gas_potential = 0
        
        # Gas potential
        self.gas_potential = 0
        self.gas_potential_before = 0
        
        # Max and min gas potential
        self.max_gas_potential = 0
        self.min_gas_potential = 0

        # Initializing can with port 0
        self.can_send = SendCan("can0")


    def set_max_gas_potential(self, max_potential) -> None:
        self.max_gas_potential = max_potential

    def set_min_gas_potential(self, min_potential) -> None:
        self.min_gas_potential = min_potential

    def get_gas_potential(self) -> float:
        try:
            gas_potential = self.potential.get_gas_pot()
            #print("GAS POT. BEF: " + str(gas_potential))
            
            #A higher offset == less prone to noise
            gas_offset = 5

            gas_potential = round((gas_potential - (self.min_gas_potential + gas_offset))/(self.max_gas_potential - self.min_gas_potential),2) #- float(start_potential)
            if gas_potential > 1:
                gas_potential = 1
            if gas_potential < 0:
                    gas_potential = 0
            

            self.gas_potential_before = gas_potential
            return gas_potential
        except Exception as e:
            print("FAILED GET GAS: " + str(e))
            self.port.read_all()
            return self.gas_potential_before

    def send_gas(self) -> None:
        self.can_send.send_bus_voltage()
        if self.drive_state == 0:
            try:
                self.can_send.send_msg(0x501, self.NEUTRAL_ARR)
                #print("Current potential: " + str(self.gas_potential))
                #print("PASS POTENTIO")
            except Exception as e:
                pass
                print("FAILED NEUTRAL")
        else:
            current_gas_pot = self.get_gas_potential()
            print("Current potential: " + str(current_gas_pot))
            VELOCITY_ARR = self.decrypt.float_to_hex_msg(current_gas_pot)


            #print(self.DRIVE_ARR)                
            
            if self.drive_state == 1 and self.is_braking() == False:
                
                for i in range(4,len(self.DRIVE_ARR)):
                    self.DRIVE_ARR[i] = VELOCITY_ARR[len(self.DRIVE_ARR) - 1 - i]
                print("IS DRIVING POT: " + str(current_gas_pot))
                #Sending potentiometer value on can
                try:
                    self.can_send.send_msg(0x501, self.DRIVE_ARR)
                    #print("PASS POTENTIO")
                except Exception as e:
                    pass
                    print("FAILED DRIVE_ARR")
                
            elif self.drive_state == 2 and self.is_braking() == False:

                for i in range(4,len(self.REVERSE_ARR)):
                    self.REVERSE_ARR[i] = VELOCITY_ARR[len(self.REVERSE_ARR)-1 - i]
                    
                try:
                    self.can_send.send_msg(0x501, self.REVERSE_ARR)
                        #print("PASS POTENTIO")
                except Exception as e:
                    pass
                    print("FAILED REVERSE")

        self.update_data()

    def is_braking(self) -> bool:
        if self.BRAKE_ARR[0] != 0:
            return True
        else:
            return False

    def reset_min_max(self) -> None:
        self.min_gas_potential = 0.0
        self.max_gas_potential = 0.0

    def set_neutral(self):
        self.drive_state = 0
        
    def set_drive(self):
        self.drive_state = 1

    def set_reverse(self):
        self.drive_state = 2

    def update_data(self):
        self.can_data.DRIVE_ARR = self.DRIVE_ARR
        self.can_data.REVERSE_ARR = self.NEUTRAL_ARR
        self.can_data.notify()

    def fetch_data(self):
        self.BRAKE_ARR = self.can_data.NEUTRAL_ARR

############################ /DRIVE ###################################

######################## brake ########################################

class BrakeCan():
    
    def __init__(self, can_data: CanData, read_can: ReaderCan) -> None:
        self.potential = ReceivePotential()
        self.decrypt = decrypt()

        self.can_data = can_data
        self.read_can = read_can

        # Drive, reverse, brake and neutral arrays
        self.DRIVE_ARR = can_data.DRIVE_ARR
        self.REVERSE_ARR = can_data.REVERSE_ARR
        self.NEUTRAL_ARR = can_data.NEUTRAL_ARR
        self.BRAKE_ARR = can_data.BRAKE_ARR

        # Drive state (0 = neutral, 1 = drive, 2 = reverse)
        self.drive_state = 0
        self.brake_potential = 0
        self.brake_potential_before = 0
        
        # MAX/MIN GAS brake POTENTIALS#
        self.max_brake_potential = 0
        self.min_brake_potential = 0

        self.send_can = SendCan("can0")

    def get_brake_potential(self) -> float:
        try:
            brake_potential = self.potential.get_brake_pot()

            # THe higher offset = lower noise
            brake_offset = 0

            #Setting max brake allowed
            max_brake_allowed = 0.2

            brake_potential = round((brake_potential - (self.min_brake_potential + brake_offset))/(self.max_brake_potential - self.min_brake_potential),2) #- float(start_potential)
                
            if brake_potential > 1:
                brake_potential = 1
            if brake_potential < 0:
                brake_potential = 0

            if brake_potential > self.brake_potential_before - 0.2 or self.read_can.bus_current > -50:
                print("Current bus current: " + str(self.read_can.bus_current))
                brake_potential = self.brake_potential_before
            
            self.brake_potential_before = brake_potential
            return brake_potential
        except Exception as e:
            print("FAILED READING brake POTENTIAL" + str(e))
            return self.brake_potential_before

    def send_brake(self):
        brake_potential = self.get_brake_potential()
        if self.is_driving() == True:
            return
        else:
            VELOCITY_ARR = self.decrypt.float_to_hex_msg(brake_potential)
            for i in range(len(VELOCITY_ARR)):
                self.BRAKE_ARR[i] = VELOCITY_ARR[len(VELOCITY_ARR) - 1 - i]
            

            print("IS BRAKING POT: " + str(brake_potential) + "\n" + "BIT DATA: " + str(self.BRAKE_ARR))
        self.update_data()


    def is_driving(self):
        last_index = len(self.DRIVE_ARR) - 1
        if self.DRIVE_ARR[last_index] != 0 or self.REVERSE_ARR[last_index] != 0:
            return True
        else:
            return False

    def set_max_brake_potential(self, max_potential) -> None:
        self.max_brake_potential = max_potential

    def set_min_brake_potential(self, min_potential) -> None:
        self.min_brake_potential = min_potential

    def reset_min_max(self) -> None:
        self.min_brake_potential = 0.0
        self.max_brake_potential = 0.0

    def set_neutral(self):
        self.drive_state = 0
        
    def set_drive(self):
        self.drive_state = 1

    def set_reverse(self):
        self.drive_state = 2

    def update_data(self):
        self.can_data.BRAKE_ARR = self.BRAKE_ARR
        self.can_data.notify()

    def fetch_data(self):
        self.DRIVE_ARR = self.can_data.DRIVE_ARR
        self.REVERSE_ARR = self.can_data.REVERSE_ARR
    

######################## /brake ########################################


class DriveCar(CanData):

    def __init__(self) -> None:
        CanData.__init__(self)
        
        self.brake_can = BrakeCan(self, None)
        self.drive_can = DriveCan(self)
        self.attach(self.brake_can)
        self.attach(self.drive_can)
    
    def drive(self):
        self.brake_can.send_brake()
        self.drive_can.send_gas()

    def get_brake_instance(self):
        return self.brake_can
    
    def get_drive_instance(self):
        return self.drive_can
    

if __name__ == "__main__":
    pass
    #drive = DriveCan()
    #drive.max_gas_potential = 600
    #drive.min_gas_potential = 300
    #drive.set_drive()

    car = DriveCar()
    car.brake_can.max_brake_potential = 600
    car.brake_can.max_brake_potential = 300
    #car.drive_can.max_gas_potential = 600
    #car.drive_can.min_gas_potential = 300
    while 1:
        #schedule.run_pending()
        #send.update_potentials()
        car.brake_can.send_brake()
        #car.drive_can.send_gas()




