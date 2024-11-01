from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import NoTransition

from widgets import SPIScreenManager

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
            text: 'SPI Process has exited unexpectedly. Check the hardware and press the button below to try to restart.'
        Button:
            text: 'Restart'
            id: restart_button
        
<ControlInterface>:
    BoxLayout:
        orientation: 'vertical'
        id: layout
        Label:
            text: 'Loaded!'
""") # TODO maybe move this to a file
class RobotGui(App):
    def build(self):
        self.rw = SPIScreenManager(transition=NoTransition())
        return self.rw
    
    def on_stop(self):
        self.rw.stop_spi()
        return super().on_stop()
    
if __name__ == '__main__':
    RobotGui().run()