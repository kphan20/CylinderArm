import os
os.environ['KIVY_NO_ARGS'] = "1"

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import NoTransition
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.config import Config
import argparse

import rclpy

from cylinder_arm_py_pkg.robot_interaction.widgets import SPIScreenManager

"""
After reading config file about actuators:

Use sliders for numeric ranges
Use toggle buttons for on/off stuff
"""

Builder.load_string("""
<InitScreen>:
    BoxLayout:
        Label:
            text: 'Waiting for initialization...'
            id: label
<FailureScreen>
    BoxLayout:
        orientation: 'vertical'
        Label:
            id: info_text
            text: 'SPI Process has exited unexpectedly. Check the hardware and press the button below to try to restart.'
        Button:
            text: 'Restart'
            id: restart_button
<ControlInterface>:
    FloatLayout:
        id: float_layout
        BoxLayout:
            orientation: 'vertical'
            id: box_layout
            Label:
                text: 'Gui Controls'
""") # TODO maybe move this to a file
class RobotGui(App):
    def __init__(self, config_file=None, **kwargs):
        super().__init__(**kwargs)
        self.config_file = config_file

    def build(self):
        self.app_layout = FloatLayout()
        self.rw = SPIScreenManager(1, self.config_file, transition=NoTransition())
        self.exit_button = Button(text="Quit", size_hint=(0.1, 0.1), pos_hint={"right":1, "top":1})
        self.exit_button.bind(on_press=lambda dt: self.stop())
        self.app_layout.add_widget(self.rw)
        self.app_layout.add_widget(self.exit_button)
        return self.app_layout

    def on_stop(self):
        if rclpy.ok():
            self.rw.stop_spi()
            rclpy.shutdown()
        return super().on_stop()
    
    def run(self):
        try:
            super().run()
        except KeyboardInterrupt:
            pass
        except Exception as e: # TODO for now have general exception
            print(e)
            self._stop()

def is_wsl():
    try:
        with open("/proc/version", "r") as file:
            version_info = file.read().lower()
            return "microsoft" in version_info
    except FileNotFoundError:
        return False

def main(args=None):
    
    if not is_wsl():
        Config.set('input', 'mouse', 'mouse,disable_multitouch')
        Config.set('graphics', 'show_cursor', '0')
        Config.set('graphics', 'maxfps', '30')
        Config.set('graphics', 'fullscreen', 'auto')
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-cf", "--config-file", default=None, help="Gui Config File Path")
    args = parser.parse_args()

    RobotGui(args.config_file).run()

if __name__ == '__main__':
    main()