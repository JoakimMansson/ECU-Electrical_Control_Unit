import os
#os.environ['KIVY_IMAGE'] = 'pil,sdl2'

import kivy
##kivy.require('1.11.1') # replace with your current kivy version !
     
import urllib
import time

from kivy.clock import Clock
from kivy.properties import ObjectProperty, NumericProperty, BoundedNumericProperty, StringProperty
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window

from kivymd.app import MDApp
from kivymd.toast import toast
from kivymd.uix.button import MDIconButton, MDFillRoundFlatButton
from kivymd.uix.dialog import MDDialog
from kivymd.uix.menu import MDDropdownMenu

from kivy.config import Config

from kivy.app import App
from kivy.clock import Clock
from kivy.properties import NumericProperty
from kivy.properties import StringProperty
from kivy.properties import BoundedNumericProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.uix.scatter import Scatter
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.uix.progressbar import ProgressBar
from os.path import join, dirname, abspath
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.core.window import Window
from urllib import request

import Initialize_CAN as init
from CAN_receiver import ReaderCan
from CAN_sender import DriveCan
from CAN_sender import BrakeCan
from CAN_sender import RawCan
from CAN_sender import DriveCar

init.initialize_CAN()
can_read = ReaderCan()
car_drive = DriveCar()
can_drive = car_drive.get_drive_instance()
can_brake = car_drive.get_brake_instance()
can_raw = RawCan()

class StartScreen(Screen):
    def __init__(self, **kw):
        super().__init__(**kw)

        # By default updates database
        self.remote_on = False
    
    def on_pre_enter(self):
       self.upd_pot = Clock.schedule_interval(lambda upd_pot: self.update_raw_potential(), 0.003)
    
    def on_leave(self):
        self.upd_pot.cancel()

    def toggle_remote(self):
        can_read.toggle_remote()

        if self.remote_on == False:
            self.remote_btn.line_color = (1,0,0,1)
            self.remote_btn.text_color = (1,0,0,1)
            self.remote_on = False
        else:
            self.remote_btn.line_color = (1,1,1,1)
            self.remote_btn.text_color = (1,1,1,1)
            self.remote_on = True


    def update_raw_potential(self):
        can_raw.update_potentials()
        gas_pot = can_raw.get_raw_gas_potential()
        brake_pot = can_raw.get_raw_brake_potential()
        self.current_gas_pot.text = str(gas_pot)
        self.current_brake_pot.text = str(brake_pot)

        max_gas, min_gas = can_raw.get_min_max_gas()
        self.min_gas_pot.text = str(min_gas)
        self.max_gas_pot.text = str(max_gas)

        max_brake, min_brake = can_raw.get_min_max_brake()
        self.min_brake_pot.text = str(min_brake)
        self.max_brake_pot.text = str(max_brake)
    
        
    def reset_potentials(self):
        can_raw.reset_min_max()
            

    # Switches to drive screen
    def switch_to_main(self):
        if self.all_potentials_set():
            self.set_min_max()
            self.manager.current = "Main"
        else:
            toast("Set a potential")
    
    # Sets min and max in DriveCan and brakeCan classes
    def set_min_max(self):
        max_gas, min_gas = can_raw.get_min_max_gas()
        can_drive.set_max_gas_potential(max_gas)
        can_drive.set_min_gas_potential(min_gas)

        max_brake, min_brake = can_raw.get_min_max_brake()
        can_brake.set_max_brake_potential(max_brake)
        can_brake.set_min_brake_potential(min_brake)        

    def all_potentials_set(self):
        max_gas, min_gas = can_raw.get_min_max_gas()
        max_brake, min_brake = can_raw.get_min_max_brake()

        if max_gas > min_gas + 10 and max_brake > min_brake + 10:
            return True
        else:
            return False
            
    def exit_script(self):
        os._exit(0)

class MainScreen(Screen):
    
    current_speed = ObjectProperty(None)
    reverseB = ObjectProperty(None)
    neutralB = ObjectProperty(None)
    driveB = ObjectProperty(None)
    
    bms_temp_img = ObjectProperty(None)
    motor_temp_img = ObjectProperty(None)
    heat_sink_temp_img = ObjectProperty(None)
    
    remote_logging = ObjectProperty(None)


    def __init__(self, **kw):
        super().__init__(**kw)

        self.current_drive_state = 0
        self.vehicle_velocity = 0
        self.double_tap = False
        
        #For toggling data to off in beginning of program
        self.showing_data = False
        Clock.schedule_once(lambda toggle_data: self.toggle_data(), 1/60)
        
        #Check temperatures and updates warning symbol
        Clock.schedule_interval(lambda update_signs: self.set_warning_signs(), 2)

    def on_pre_enter(self):
        # Updates speedometer
        self.upd_speedometer = Clock.schedule_interval(lambda speedometer: self.set_current_velocity(int(can_read.vehicle_velocity)), 0.200)
        
        # Reads potentiometer and sends a velocity or brake
        self.send_gasNbreak = Clock.schedule_interval(lambda output_speed: self.send_gas_brake(), 0.003)

        #Update voltage and current for MC & BMS
        self.update_MC_BMS = Clock.schedule_interval(lambda update_values: self.update_V_C_BMS_MC(), 0.5)

    def on_pre_leave(self):
        self.send_gasNbreak.cancel()
        self.upd_speedometer.cancel()
        self.update_MC_BMS.cancel()
        
    def send_gas_brake(self) -> None:
        can_drive.send_gas()
        can_brake.send_brake()

    def set_current_velocity(self, velocity) -> None:
        self.current_vehicle_velocity = velocity
        self.current_speed.text = str(velocity)
        
    
    def update_V_C_BMS_MC(self):
        self.mcCurrentT.text = "MC A: " + str(can_read.bus_current)
        self.mcVoltageT.text = "MC V: " + str(can_read.bus_voltage)
        self.bmsCurrentT.text = "BSM A: " + str(can_read.pack_current)
        self.bmsVoltageT.text = "BSM V: " + str(can_read.pack_voltage) 

    def set_warning_signs(self):
        MT = int(can_read.motor_temp)
        HST = int(can_read.heat_sink_temp)
        BMS = int(can_read.internal_temp)
        
        if MT < 50:
            self.motor_temp_img.source = "indication_signals/green.png"
        elif 50 <= MT < 70:
            self.motor_temp_img.source = "indication_signals/yellow.png"
        else:
            self.motor_temp_img.source = "indication_signals/red.png"
            
        if HST < 50:
            self.heat_sink_temp_img.source = "indication_signals/green.png"
        elif 50 <= HST < 70:
            self.heat_sink_temp_img.source = "indication_signals/yellow.png"
        else:
            self.heat_sink_temp_img.source = "indication_signals/red.png"
            
        if BMS < 50:
            self.bms_temp_img.source = "indication_signals/green.png"
        elif 50 <= HST < 70:
            self.bms_temp_img.source = "indication_signals/yellow.png"
        else:
            self.bms_temp_img.source = "indication_signals/red.png"
            
        
    
    def to_set_pot_screen(self):
        if self.current_drive_state == 0:
            self.manager.current = "Start"
        else:
            toast("Change to neutral")

    def reverse(self):
        if self.current_drive_state == 1 and self.vehicle_velocity > 0.5:
            toast("Slow down before setting reverse")
        else:
            self.reverseB.line_color = (0/255, 109/255, 176/255, 1)
            self.reverseB.text_color = (0/255, 109/255, 176/255, 1)

            self.driveB.line_color = (1, 1, 1, 1)
            self.driveB.text_color = (1, 1, 1, 1)

            self.neutralB.line_color = (1, 1, 1, 1)
            self.neutralB.text_color = (1, 1, 1, 1)
            
            self.set_reverse()


    def drive(self):
        if self.current_drive_state == 2 and self.vehicle_velocity < -0.5:
            toast("Slow down before setting drive")
        else:
            self.driveB.line_color = (0 / 255, 109 / 255, 176 / 255, 1)
            self.driveB.text_color = (0 / 255, 109 / 255, 176 / 255, 1)

            self.reverseB.line_color = (1, 1, 1, 1)
            self.reverseB.text_color = (1, 1, 1, 1)

            self.neutralB.line_color = (1, 1, 1, 1)
            self.neutralB.text_color = (1, 1, 1, 1)
        
            self.set_drive()


    def neutral(self):
        self.neutralB.line_color = (0 / 255, 109 / 255, 176 / 255, 1)
        self.neutralB.text_color = (0 / 255, 109 / 255, 176 / 255, 1)

        self.reverseB.line_color = (1, 1, 1, 1)
        self.reverseB.text_color = (1, 1, 1, 1)

        self.driveB.line_color = (1, 1, 1, 1)
        self.driveB.text_color = (1, 1, 1, 1)
        
        self.set_neutral()
        
    def toggle_data(self):
        
        if self.double_tap == True:
            
            if self.showing_data == False:
                self.tempT.color, self.motorT.color, self.bmsT.color, self.hsT.color = (0 / 255, 109 / 255, 176 / 255, 1), (1, 1, 1, 1), (1, 1, 1, 1), (1, 1, 1, 1)
                self.dataT.color, self.remoteT.color = (0 / 255, 109 / 255, 176 / 255, 1), (1, 1, 1, 1)
                self.motor_temp_img.size_hint_y, self.bms_temp_img.size_hint_y, self.heat_sink_temp_img.size_hint_y = 0.05, 0.05, 0.05
                self.remote_logging.size_hint_y = 0.05
                self.dataB.line_color, self.dataB.text_color = (0 / 255, 109 / 255, 176 / 255, 1), (0 / 255, 109 / 255, 176 / 255, 1)

                self.mc_bms.color = (0 / 255, 109 / 255, 176 / 255, 1)
                self.bmsVoltageT.color, self.bmsCurrentT.color = (1, 1, 1, 1), (1, 1, 1, 1)
                self.mcVoltageT.color, self.mcCurrentT.color = (1, 1, 1, 1), (1, 1, 1, 1)

                self.showing_data = True
                self.double_tap = False
            else:
                self.tempT.color, self.motorT.color, self.bmsT.color, self.hsT.color = (1, 1, 1, 0), (1, 1, 1, 0), (1, 1, 1, 0), (1, 1, 1, 0)
                self.dataT.color, self.remoteT.color= (1, 1, 1, 0), (1, 1, 1, 0)
                self.motor_temp_img.size_hint_y, self.bms_temp_img.size_hint_y, self.heat_sink_temp_img.size_hint_y = 0, 0, 0
                self.remote_logging.size_hint_y = 0
                self.dataB.line_color, self.dataB.text_color = (1, 1, 1, 1), (1, 1, 1, 1)

                self.mc_bms.color = (0 / 255, 109 / 255, 176 / 255, 0)
                self.bmsVoltageT.color, self.bmsCurrentT.color = (1, 1, 1, 0), (1, 1, 1, 0)
                self.mcVoltageT.color, self.mcCurrentT.color = (1, 1, 1, 0), (1, 1, 1, 0)

                self.showing_data = False
                self.double_tap = False
        else:
            self.double_tap = True
        

    def set_neutral(self):
        can_brake.set_neutral()
        can_drive.set_neutral()
        
    def set_drive(self):
        can_brake.set_drive()
        can_drive.set_drive()

    def set_reverse(self):
        can_brake.set_reverse()
        can_drive.set_reverse()

class WindowManager(ScreenManager):
    enter_name_screen = ObjectProperty(None)
    connect_screen = ObjectProperty(None)
    home_screen = ObjectProperty(None)


class AlarmApp(MDApp):

    def build(self):
        kv = Builder.load_file(os.path.realpath("my.kv"))
        Window.size = (600, 480)
        Window.fullscreen = "auto"
        Window.show_cursor = False
        return kv


if __name__ == "__main__":
    AlarmApp().run()
            