import os
import can
import requests
from datetime import datetime
import time
import re
import math
from threading import Thread
from Database_MDB import database
from Database_SQL import LocalDatabase

os.system("sudo ifconfig can1 up")
os.system("sudo ifconfig can0 up")


can0 = can.interface.Bus(bustype = 'socketcan', channel = 'can0')
can1 = can.interface.Bus(bustype = 'socketcan', channel = 'can1')

cluster = db_adress
collection = "remote_can_statistics"
dataB = "can"
db = database(cluster, collection, dataB)

# MC - Motor Control
motor_velocity = 0
vehicle_velocity = 0
bus_voltage = 0
bus_current = 0
motor_temp = 0
heat_sink_temp = 0
acceleration = 0

#BMS - Battery management system

class ReaderCan():
    
    def __init__(self):
        cluster = "mongodb+srv://whit3eo:aoiP8mHeZBoZ8tuu@alarmcluster.y3bqh.mongodb.net/?retryWrites=true&w=majority"
        collection = "remote_can_statistics"
        dataB = "can"
        self.db = database(cluster, collection, dataB)

            
        # MC - Motor Control
        self.motor_velocity = 0.0
        self.vehicle_velocity = 0.0
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.motor_temp = 0.0
        self.heat_sink_temp = 0.0
        self.acceleration = 0.0
        self.power_loss = 0.0
        
        #BMS - Battery management system
        self.avg_cell_voltage = 0.0
        self.high_cell_voltage = 0.0
        self.low_cell_voltage = 0.0
        self.pack_current = 0.0
        self.pack_voltage = 0.0
        
        self.avg_temp = 0.0
        self.high_temp = 0.0
        self.low_temp = 0.0
        self.internal_temp = 0.0
        self.state_of_charge = 0.0
        self.avg_pack_current = 0.0

        #Stop sending to database
        self.stop_updating_db = True
        self.read_can()
                
    
    def read_can(self):
        MainT = Thread(target=self.read_can)
        can_msg = re.sub(" ", "", str(can1.recv(2)))
        #print(can_msg)
        can_id = can_msg[31:34]
        can_data = can_msg[41:57]
        #print("ID: " + can_id + " " + "CAN data: " + can_data)
        self.decrypt(can_id, can_data)
        time.sleep(0.01)
        MainT.start()

    def toggle_remote(self) -> None:
        if self.stop_updating_db:
            self.stop_updating_db = False
        else:
            self.stop_updating_db = True

    def decrypt(self, can_ID, can_data) -> None:

        if can_ID == "402":
            BV, BC = self.hex_to_bin(can_data)
            self.bus_voltage = self.IEEE754(BV)
            self.bus_current = self.IEEE754(BC)
            #print("Voltage: " + str(self.bus_voltage), "Current: " + str(self.bus_current))
            power_loss = self.calculate_power_loss(self.bus_current, self.bus_voltage)
            
            acc_thread = Thread(target=self.update_acceleration, args=(1,))
            acc_thread.start()
            efficiency = self.calculate_efficiency(self.bus_current, self.bus_voltage, self.acceleration)
            #print("acceleration: " + str(acceleration))
            try:
                BV_thread = Thread(target=self.upload_data_db, args=("bus_voltage", self.bus_voltage,))
                BC_thread = Thread(target=self.upload_data_db, args=("bus_current", self.bus_current,))
                EFF_thread = Thread(target=self.upload_data_db, args=("motor_efficiency", efficiency,))
                PL_thread = Thread(target=self.upload_data_db, args=("power_loss_mc", power_loss,))
                #ACC_thread = Thread(target=self.upload_data_db, args=("acceleration", self.acceleration,))
                
            
                #ACC_thread.start()
                PL_thread.start()
                BV_thread.start()
                BC_thread.start()
                EFF_thread.start()
            except Exception as e:
                print("Failed to update DB (current, voltage, acceleration)")
    
        if can_ID == "403":
            MV, VV = self.hex_to_bin(can_data)
            #self.motor_velocity = self.IEEE754(MV)
            self.vehicle_velocity = self.IEEE754(VV)*3.6
            #print("Motor velocity: " + str(self.motor_velocity), "Vehicle velocity: " + str(self.vehicle_velocity))
            
            try:
                MV_thread = Thread(target=self.upload_data_db, args=("motor_velocity", self.motor_velocity,))
                #VV_thread = Thread(target=self.upload_data_db, args=("vehicle_velocity", self.vehicle_velocity,))
            
                MV_thread.start()
                #VV_thread.start()                   
                        
            except Exception as e:
                print("Failed to update DB (motor_velocity, vehicle_velocity)")
    
        if can_ID == "40b":
            MT, HST = self.hex_to_bin(can_data)
            self.motor_temp = self.IEEE754(MT)
            self.heat_sink_temp = self.IEEE754(HST)
            #print("Motor temp: " + str(motor_temp) + ", " + "Heatsink temp: " + str(heat_sink_temp))
            try:
                HST_thread = Thread(target=self.upload_data_db, args=("heat_sink_temp", self.heat_sink_temp,))
                MT_thread = Thread(target=self.upload_data_db, args=("motor_temp", self.motor_temp,))
            
                HST_thread.start()
                MT_thread.start()  
            except Exception as e:
                print("Failed to update DB (motor_temp, heat_sink_temp)")


        if can_ID == "600":
            self.avg_cell_voltage = int(can_data[0:4], 16)*10**(-4)
            self.high_cell_voltage = int(can_data[4:8], 16)*10**(-4)
            self.low_cell_voltage = int(can_data[8:12], 16)*10**(-4)
            self.pack_current = int(can_data[12:16], 16)*10**(-1)
            
            
            #print("BMS 600: " + str(can_data))
            #print("avg_cellvolt: " + str(self.avg_cell_voltage))
            #print("high_cellvolt: " + str(self.high_cell_voltage))
            #print("min_cellvolt: " + str(self.low_cell_voltage))
            #print("pack_current: " + str(self.pack_current))
            try:
                ACV_thread = Thread(target=self.upload_data_db, args=("avg_cellvolt", self.avg_cell_voltage,))
                HCV_thread = Thread(target=self.upload_data_db, args=("high_cellvolt", self.high_cell_voltage,))
                LCV_thread = Thread(target=self.upload_data_db, args=("low_cellvolt", self.low_cell_voltage,))
                PC_thread = Thread(target=self.upload_data_db, args=("pack_current", self.pack_current,))
            
                ACV_thread.start()
                HCV_thread.start()
                LCV_thread.start()
                PC_thread.start()  
            except Exception as e:
                print("Failed to update DB (0x600)")
        
        if can_ID == "601":

            #print("CAN data: " + can_data)
            #print("BMS 601: " + str(can_data))
            self.avg_temp = int(can_data[0:2], 16)
            self.high_temp = int(can_data[2:4], 16)
            self.low_temp = int(can_data[4:6], 16)
            self.internal_temp = int(can_data[6:8], 16)
            self.state_of_charge = int(can_data[8:10], 16)*0.5
            self.avg_pack_current = 0
            self.pack_voltage = int(can_data[10:14], 16)*0.1
            
            
            try:
                AT_thread = Thread(target=self.upload_data_db, args=("avg_temp", self.avg_temp,))
                HT_thread = Thread(target=self.upload_data_db, args=("high_temp", self.high_temp,))
                LT_thread = Thread(target=self.upload_data_db, args=("low_temp", self.low_temp,))
                IT_thread = Thread(target=self.upload_data_db, args=("internal_temp", self.internal_temp,))
                SOC_thread = Thread(target=self.upload_data_db, args=("pack_SOC", self.state_of_charge,))
                APC_thread = Thread(target=self.upload_data_db, args=("avg_current", self.avg_pack_current,))
                PV_thread = Thread(target=self.upload_data_db, args=("pack_voltage", self.pack_voltage,))
                
                AT_thread.start()
                HT_thread.start()
                LT_thread.start()
                IT_thread.start()
                SOC_thread.start()
                APC_thread.start()
                PV_thread.start()
            except Exception as e:
                print("Failed to update DB (0x601)")
            #print("avg_temp: " + str(self.avg_temp))
            #print("high_temp: " + str(self.high_temp))
            #print("low_temp: " + str(self.low_temp))
            #print("internal_temp: " + str(self.internal_temp))
            #print("state_of_charge: " + str(self.state_of_charge))
            #print("avg_pack_current: " + str(self.avg_pack_current))
        
    def IEEE754(self, bin_value: bin) -> float:

        if str(bin_value) == 32*"0":
            return 0.0
    
        exponent_bits = bin_value[1:9]
        mantissa_bits = bin_value[9:]
        #print("num: " + bin_value)
        #print("exponent: " + exponent_bits)
        #print("mantissa: " + mantissa_bits)

        mantissa_decimal = 0
        exponent_decimal = 0

        #Calculating mantissa
        for i in range(len(mantissa_bits)):
            if int(mantissa_bits[i]) != 0:
                mantissa_decimal = mantissa_decimal + math.pow(2, -(i+1))

        #Calculating exponent
        exponent_counter = 0
        for i in range(len(exponent_bits)-1, -1, -1):

            if int(exponent_bits[i]) != 0:
                exponent_decimal = exponent_decimal + math.pow(2, exponent_counter)

            exponent_counter = exponent_counter + 1

        return (1 + mantissa_decimal) * math.pow(2, exponent_decimal - 127)


    def hex_to_bin(self, hex_value: str):
    
        new_string = ""
        for i in range(0, len(hex_value), 2):
            if i + 2 <= len(hex_value):
                new_string = hex_value[i:i+2] + new_string
    
        hex_value = new_string

        #31 .. 0 bits
        hex1 = hex_value[8:16]
        #print("hex1: " + hex1)
        bin1 = bin(int("1" + hex1,16))[3:]
        #32 .. 63 bits
        hex2 = hex_value[0:8]
        #print("hex2: " + hex2)
        bin2 = bin(int("1" + hex2, 16))[3:]
        #print("bin1: " + str(bin1))
        #print("bin2: " + str(bin2))
        #print("bin1: " + str(bin1) + ", " + "bin2: " + str(bin2))
        return bin1, bin2

    def calculate_power_loss(self, bus_current: float, bus_voltage: float):
            Req = 1.08*(10**(-2))
            alpha = 3.345*(10**(-3))
            beta = 1.8153*(10**(-2))
            CFeq = 1.5625*(10**(-4))
            
            power_loss = Req*(bus_current**2) + bus_voltage*(alpha*bus_current + beta) + CFeq*(bus_voltage**2)
            #print(power_loss)
            return power_loss
            
    # Giving the proportion of the acceleration/(bus_voltage*bus_current)
    def calculate_efficiency(self, bus_current: float, bus_voltage: float, acceleration: float) -> float:
        energy_in = bus_current * bus_voltage
        if energy_in == 0:
            return 0
        else:
            return acceleration/energy_in

    # @time_period - the period in which you want
    # to calculate the speed difference between
    # NOTE HAVE TO USE THREADING ON THIS FUNCTION
    # SINCE time.sleep() WILL DISRUPT A LOT
    def update_acceleration(self, time_period: int):
        global acceleration
        can_msg = " "*36
        while(can_msg[31:34] != "403"):
            can_msg = re.sub(" ", "", str(can1.recv(1)))
        
        can_data = can_msg[41:57]
    
        MV, start_vehicle_velocity = self.hex_to_bin(can_data)
        start_vehicle_velocity = self.IEEE754(start_vehicle_velocity)
        #print("start_vehicle_velocity: " + str(start_vehicle_velocity))
    
        start_time = time.time()
        time.sleep(time_period)
    
        can_msg = " "*36
        while(can_msg[31:34] != "403"):
            can_msg = re.sub(" ", "", str(can1.recv(1)))
    
        can_data = can_msg[41:57]
    
        MV, end_vehicle_velocity = self.hex_to_bin(can_data)
        end_vehicle_velocity = self.IEEE754(end_vehicle_velocity)
        #print("end_vehicle_velocity: " + str(end_vehicle_velocity))
        
        end_time = time.time()
        acceleration = (end_vehicle_velocity - start_vehicle_velocity)/(end_time - start_time)
        #print("acc: " + str(acceleration))
        self.acceleration = acceleration
# 
    # Thread this function for increased performance
    def upload_data_db(self, query_variable: str, variable_name) -> None:
        if self.stop_updating_db == True:
            return
        else:
            self.db.update_element(0, query_variable, float(variable_name))
        
if __name__ == "__main__":
    pass
    read = ReaderCan()
    
    #os.system('sudo ifconfig can0 down')
    #os.system('sudo ifconfig can1 down')


    