from typing import Callable

from cylinder_arm_interfaces.msg import SPISend

class ActuatorEventManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, send_spi_message: Callable[[SPISend], None]=None):
        if not hasattr(self, "_initialized"):
            self._initialized = True
            self.send_spi_message = send_spi_message
    
    @staticmethod
    def convert_to_bytes(val: int):
        if val == 0:
            msg = [0]
        else:
            msg = list(val.to_bytes((val.bit_length() + 7)//8))
        return msg