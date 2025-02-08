from kivy.uix.slider import Slider
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.logger import Logger
from kivy.uix.button import Button
import json
from queue import Queue
from threading import Event, Lock
from functools import partial
import re

from rclpy import spin_once, ok
from .messages import MESSAGES
from .robot_threading import ThreadingDict

from cylinder_arm_interfaces.msg import SPISend
from .touchscreen_node import TouchScreenNode
from .event import ActuatorEventManager

class RobotValue:
    def __init__(self, val):
        self.lock = Lock()
        self.value: int = val

    def get(self):
        with self.lock:
            return self.value
        
    def set(self, val):
        with self.lock:
            self.value = val

class RobotSliderInput(BoxLayout):
    def __init__(self, name, delta: ThreadingDict, actuator_id, **kwargs):
        super().__init__()

        self.slider = RobotSlider(name, delta, actuator_id, **kwargs)
        self.text = RobotInput(self.slider.min, self.slider.max)

        self.add_widget(self.slider)
        self.add_widget(self.text)

        self.slider.bind(value=self.on_slider_edit)
        self.text.bind(text=self.on_text_edit)
    
    def on_slider_edit(self, w, val):
        self.text.edit(self.slider.value)
    
    def on_text_edit(self, w, val):
        if val == '':
            return
        true_slider_value = self.slider.edit(int(float(val)))
        self.text.text = str(true_slider_value)

class RobotInput(TextInput):
    def __init__(self, min_val: int, max_val: int, **kwargs):
        super().__init__(**kwargs)
        self.regexp = re.compile('[0-9][0-9]*')
        self.min_val = min_val
        self.max_val = max_val

    def insert_text(self, substring, from_undo=False):
        if not self.regexp.fullmatch(substring):
            substring = 0
        else:
            substring = str(min(max(int(substring), self.min_val), self.max_val))
        return super().insert_text(substring, from_undo)

    def edit(self, value):
        self.text = str(value)

class RobotSlider(Slider):
    def __init__(self, name, delta: ThreadingDict, actuator_id, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta
        self.msg = SPISend()
        self.msg.actuator_id = actuator_id
        self.msg.status_message = MESSAGES["ready"]

    def on_value(self, w, touch):
        self.delta.update(self.name, self.value)
        self.send_message(int(self.value))
    
    def send_message(self, val: int):
        self.msg.message = ActuatorEventManager.convert_to_bytes(val)
        ActuatorEventManager().send_spi_message(self.msg)

    def edit(self, value):
        if value >= self.min and value <= self.max:
            self.value = value
        return self.value

class RobotSwitch(ToggleButton):
    def __init__(self, name, delta:ThreadingDict, debounce_time=0.3, **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.delta = delta
        self.text = "OFF"
        self.state = "normal"
        self.previous_press_time = Clock.get_time()
        self.debounce_time = debounce_time
    
    def _do_press(self):
        curr_time = Clock.get_time()
        if curr_time - self.previous_press_time > self.debounce_time:
            self.previous_press_time = curr_time
            super()._do_press()
        else:
            Logger.info("Debounce")
    
    def on_state(self, w, v):
        if self.state == "normal":
            self.text = "OFF"
        else:
            self.text = "ON"

class SPIScreenManager(ScreenManager):
    def __init__(self, spi_bus, config_file=None, **kwargs):
        super().__init__(**kwargs)

        self.response_q = Queue()
        self.init_event = Event()
        self.spi_failed = RobotValue(0)

        self.spi_bus = spi_bus
        self.ros_node = TouchScreenNode(spi_bus, self.init_event, self.spi_failed)
        
        self.init_screen = InitScreen(self.init_event, self.spi_failed, partial(self.ros_node.start_hardware), name="init")
        self.failure_screen = FailureScreen(name="failure")
        self.control_screen = ControlInterface(self.spi_failed, config_file, name="control")
        self.add_widget(self.init_screen)
        self.add_widget(self.failure_screen)
        self.add_widget(self.control_screen)
        self.transition_to_init()

        self.spin_check = Clock.schedule_interval(self.spin_node,0.2)
        Clock.schedule_once(lambda _ : ActuatorEventManager(partial(self.send_spi_message)), 0)

    def send_spi_message(self, msg: SPISend):
        msg.spi_bus = self.spi_bus
        self.ros_node.send_message(msg)

    def _send_spi_message(self, actuator_id=0, data=[], status=MESSAGES["ready"]):
        msg = SPISend()
        msg.spi_bus = self.spi_bus
        msg.actuator_id = actuator_id
        msg.message = data
        msg.status_message = status
        self.ros_node.send_message(msg)

    def stop_spi(self):
        # Called when kivy window is closed to stop SPI comms
        self._send_spi_message(status=MESSAGES["terminate"])
        self.ros_node.destroy_node()

    def transition_to_init(self):
        self.current = "init"
        self.init_screen.start_initialization()

    def transition_to_failure(self, is_programming=False):
        self.current = "failure"
        self.failure_screen.start_initialization(is_programming)

    def transition_to_controls(self):
        self.control_screen.start_initialization()
        self.current = "control"
    
    def spin_node(self, _):
        if ok():
            spin_once(self.ros_node, timeout_sec=0.1)


class InitScreen(Screen):
    def __init__(self, init_event: Event, spi_failed: RobotValue, start_hardware, **kw):
        super().__init__(**kw)
        self.init_event = init_event
        self.spi_failed = spi_failed
        self.start_hardware = start_hardware
        self.hw_req_future = None

    def hw_callback(self, future):
        try:
            response = future.result()
            print(f"Service response: {response}")
        except Exception as e:
            print(f"Service call failed: {e}")
        finally:
            self.hw_req_future = None

    def start_initialization(self):
        self.check_init = Clock.schedule_interval(self.check_spi,0.5)
        self.init_event.clear()
        self.spi_failed.set(0)
        self.hw_req_future = self.start_hardware()
        if self.hw_req_future is not None:
            self.hw_req_future.add_done_callback(self.hw_callback)

    def check_spi(self, dt):
        # exit early if event isn't set yet
        if not self.init_event.is_set():
            if self.hw_req_future is None:
                self.hw_req_future = self.start_hardware()
            return

        # cancel initialization check
        self.check_init.cancel()

        # go to failure screen if SPI process exited
        if self.spi_failed.get():
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
    
    def start_initialization(self, is_programming):
        self.restart_btn.disabled = False
        label = self.ids['info_text']
        if is_programming:
            label.text = 'Take this time to program your microcontrollers.'
        else:
            label.text = 'SPI Process has exited unexpectedly. Check the hardware and press the button below to try to restart.'

class ControlInterface(Screen):
    def __init__(self, spi_failed: RobotValue, config_file = None, **kw):
        super().__init__(**kw)
        self.spi_failed = spi_failed

        self.layout: BoxLayout = self.ids["box_layout"]
        # Good testing values: 
        # 0-249 for LED PWM
        # 0-4095 for AS5600 encoder
        if config_file is None:
            self.layout.add_widget(RobotSwitch(name="TestSwitch", delta=ThreadingDict()))
            self.layout.add_widget(RobotSlider("TestSlider", delta=ThreadingDict(), min=0, max=4095, step=1))
        else:
            self.load_config(config_file)

        self.program_button = Button(text="Program", size_hint=(0.1, 0.1), pos_hint={"left":1, "top":1})
        self.program_button.bind(on_press=self.program_button_action)
        self.ids["float_layout"].add_widget(self.program_button)

    def program_button_action(self, dt):
        self.check_spi_interval.cancel()
        self.manager.transition_to_failure(True)
        self.program_button.disabled = True

    def check_spi(self, _):
        # check to see if SPI has encountered errors and/or exited early
        if self.spi_failed.get():
            self.check_spi_interval.cancel()
            self.manager.transition_to_failure()

    def load_config(self, config_file):
        with open(config_file) as f:
            config = json.load(f)

        for interface in config:
            self.layout.add_widget(RobotSliderInput(interface["name"], ThreadingDict(), interface["id"], min=interface["min"], max=interface["max"], step=1))
    
    def start_initialization(self):
        # SPI process has a timeout, so Kivy client has to send heartbeat
        self.check_spi_interval = Clock.schedule_interval(self.check_spi, 1)
        self.program_button.disabled = False
