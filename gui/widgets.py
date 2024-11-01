from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.behaviors import ToggleButtonBehavior
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.logger import Logger
from kivy.uix.button import Button
import json
from multiprocessing import Queue, Event, Value
import time

from messages import MESSAGES
from spi_process import SpiProcess
from robot_threading import ThreadingDict

class RobotSlider(Slider):
    def __init__(self, name, control, delta:ThreadingDict, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta
        self.control = control

    def on_value(self, w, touch):
        self.delta.update(self.name, self.value)
        self.control.send_message(int(self.value))


class RobotSwitch(ToggleButton):
    def __init__(self, name, delta:ThreadingDict, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta
        self.text = "OFF"
        self.state = "normal"
    
    def on_state(self, w, v):
        if self.state == "normal":
            self.text = "OFF"
        else:
            self.text = "ON"

class SliderManager:
    def __init__(self, delta) -> None:
        self.sliders = {}
        self.delta = delta

    def add_slider(self, min, max, name):
        new_slider = RobotSlider(name, self.delta, min=min, max=max, value_track=True)
        self.sliders[name] = new_slider

class SPIScreenManager(ScreenManager):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.spi = None
        self.q = Queue()
        self.response_q = Queue()
        self.init_event = Event()
        self.spi_failed = Value("i", 0)
        self.init_screen = InitScreen(self.init_event, self.spi_failed, name="init")
        self.failure_screen = FailureScreen(name="failure")
        self.control_screen = ControlInterface(self.spi_failed, self.q, name="control")
        self.add_widget(self.init_screen)
        self.add_widget(self.failure_screen)
        self.add_widget(self.control_screen)
        self.transition_to_controls()

    def stop_spi(self):
        # Called when kivy window is closed to stop SPI comms
        if self.spi is not None:
            self.spi.join()

    def transition_to_init(self):
        self.current = "init"
        self.init_event.clear()
        self.init_screen.start_initialization()
        self.spi = SpiProcess(self.q, self.response_q, self.init_event, self.spi_failed)
        self.spi.start()

    def transition_to_failure(self):
        self.current = "failure"
        self.stop_spi()
        self.failure_screen.restart_btn.disabled = False

    def transition_to_controls(self):
        self.current = "control"


class InitScreen(Screen):
    def __init__(self, init_event, spi_failed, **kw):
        super().__init__(**kw)
        self.init_event = init_event
        self.spi_failed = spi_failed

    def start_initialization(self):
        self.check_init = Clock.schedule_interval(self.check_spi,0.5)

    def check_spi(self, dt):
        # exit early if event isn't set yet
        if not self.init_event.is_set():
            return

        # clear event and cancel initialization check
        self.init_event.clear()
        self.check_init.cancel()

        # go to failure screen if SPI process exited
        with self.spi_failed.get_lock():
            if self.spi_failed.value:
                self.spi_failed.value = 0
                self.manager.transition_to_failure()
                return
            
        # allow for control interface if SPI communications are up
        self.manager.transition_to_controls()


class FailureScreen(Screen):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.restart_btn: Button = self.ids["restart_button"]
        self.restart_btn.bind(on_press=self.restart_spi)

    def restart_spi(self, dt):
        self.restart_btn.disabled = True
        self.manager.transition_to_init()


class ControlInterface(Screen):
    def __init__(self, spi_failed, q: Queue, **kw):
        super().__init__(**kw)
        self.spi_failed = spi_failed
        self.q = q

        # SPI process has a timeout, so Kivy client has to send heartbeat
        self.check_spi_interval = Clock.schedule_interval(self.check_spi, 0.5)

        self.layout: BoxLayout = self.ids["layout"]
        self.layout.add_widget(RobotSlider("TestSlider", self, delta=ThreadingDict(), min=0, max=249, step=1))
        self.layout.add_widget(RobotSwitch(name="TestSwitch", delta=ThreadingDict()))

    def send_message(self, val):
        msg = list(val.to_bytes((val.bit_length() + 7)//8))
        self.q.put(msg)
        print(msg)

    def check_spi(self, dt):
        # check to see if SPI has encountered errors and/or exited early
        with self.spi_failed.get_lock():
            if self.spi_failed.value:
                self.spi_failed.value = 0
                self.manager.transition_to_failure()
                return
        # otherwise, send heartbeat
        self.q.put(MESSAGES["hb"])

    def load_config(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        for interface in config:
            pass  # TODO add logic to add widgets to this interface
