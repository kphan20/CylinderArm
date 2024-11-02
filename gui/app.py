from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import NoTransition
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.config import Config

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
    def build(self):
        self.app_layout = FloatLayout()
        self.rw = SPIScreenManager(transition=NoTransition())
        self.exit_button = Button(text="Quit", size_hint=(0.1, 0.1), pos_hint={"right":1, "top":1})
        self.exit_button.bind(on_press=self.quit_button)
        self.app_layout.add_widget(self.rw)
        self.app_layout.add_widget(self.exit_button)
        return self.app_layout

    def quit_button(self):
        self.stop()

    def on_stop(self):
        self.rw.stop_spi()
        return super().on_stop()
    
    def run(self):
        try:
            super().run()
        except: # TODO for now have general exception
            self._stop()
    
if __name__ == '__main__':
    Config.set('input', 'mouse', 'mouse,disable_multitouch')
    Config.set('graphics', 'show_cursor', '0')
    Config.set('graphics', 'maxfps', '30')
    Config.set('graphics', 'fullscreen', 'auto')
    RobotGui().run()