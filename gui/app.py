import kivy
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.behaviors import ToggleButtonBehavior
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition
from kivy.clock import Clock
from threading import Event
import json
from types import MethodType

from robot_threading import ThreadingDict
#from spi import SpiComms, CommState

"""
After reading config file about actuators:

Use sliders for numeric ranges
Use toggle buttons for on/off stuff
"""

class RobotSlider(Slider):
    def __init__(self, name, delta: ThreadingDict, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta
    
    def on_touch_move(self, touch):
        self.delta.update(self.name, self.value)
        return super().on_touch_move(touch)

class RobotSwitch(ToggleButton):
    def __init__(self, name, delta: ThreadingDict, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta

class SliderManager:
    def __init__(self, delta: ThreadingDict) -> None:
        self.sliders = {}
        self.delta = delta
    
    def add_slider(self, min, max, name):
        new_slider = RobotSlider(name, self.delta, min=min, max=max, value_track=True)
        self.sliders[name] = new_slider

class InitScreen(Screen):
    pass

class ControlInterface(Screen):
    def load_config(self, config_file):
        with open(config_file) as f:
            config = json.load(f)
        
        for interface in config:
            pass # TODO add logic to add widgets to this interface

Builder.load_string("""
<InitScreen>:
    BoxLayout:
        Label:
            text: 'Waiting for initialization...'

<ControlInterface>:
    BoxLayout:
        Label:
            text: 'Loaded!'
""") # TODO maybe move this to a file

class RobotWidget(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.delta = ThreadingDict()
        self.events = ThreadingDict()

        self.events.update("init_event", False)
        self.events.update("heartbeat_event", False)
        self.events.update("command_event", False)
        #self.events.update("curr_state", CommState.INIT)

        self.heartbeat_enabled = True
        #self.comms = SpiComms(self.delta, self.command_event, self.init_event, self.comms_init, self.heartbeat_event)

        self.init_screen = InitScreen(name="init")
        self.control_screen = ControlInterface(name="control")
        self.add_widget(self.init_screen)
        self.add_widget(self.control_screen)
    
    def initialize_config(self):
        # TODO parse the config file and add children
        self.start_comms_init()

    def disable_heartbeat(self):
        self.heartbeat_enabled = False

    def enable_heartbeat(self):
        self.heartbeat_enabled = True

    def heartbeat(self):
        if self.heartbeat_enabled:
            self.events.read_and_flip("heartbeat_event", False)
    
    def start_comms_init(self):
        self.events.read_and_flip("init_event", False)

class RobotGui(App):
    def build(self):
        rw = RobotWidget(transition=NoTransition())
        Clock.schedule_interval(lambda dt: rw.heartbeat(), 1.0/5.0)
        return rw
    
    def on_stop(self):
        return super().on_stop()
    

if __name__ == '__main__':
    RobotGui().run()