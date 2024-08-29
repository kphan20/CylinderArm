from collections.abc import Callable
from enum import Enum
from threading import Thread, Timer, Event
from typing import Any, Iterable, Mapping

from robot_threading import ThreadingDict
from config import START_BYTE, END_BYTE, HANDSHAKE_BYTE, HEARTBEAT_BYTE, COMMAND_BYTE

class CommState(Enum):
    INIT = 0
    HEARTBEAT = 1

class SpiComms(Thread):
    def __init__(self, delta: ThreadingDict, events: ThreadingDict, crc16_poly= 0x8005, group: None = None, target: Callable[..., object] | None = None, name: str | None = None, args: Iterable[Any] = ..., kwargs: Mapping[str, Any] | None = None, *, daemon: bool | None = None) -> None:
        super().__init__(group, target, name, args, kwargs, daemon=daemon)
        self.delta = delta
        self.events = events
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 0
        self.spi.mode = 0

        self.state = CommState.INIT
        self.running = True

        self.crc16_poly = crc16_poly
        self.handshake_msg = self.initiate_handshake_msg()
        self.heartbeat_msg = self.heartbeat_msg()
        self.heartbeat_fails = 0

    def run(self) -> None:
        while self.running:
            if self.state == CommState.INIT:
                if self.events.read_and_flip("init_event", True):
                    res = self.handle_handshake() # TODO handle handshake failure
                    self.state = CommState.HEARTBEAT # TODO handle failures of communication
                    self.events.update("curr_state", CommState.HEARTBEAT)
            elif self.state == CommState.HEARTBEAT:
                if self.events.read_and_flip("heartbeat_event", True):
                    self.handle_heartbeat()
                if self.events.read_and_flip("command_event", True):
                    self.handle_command()
                if self.events.read("init_event"):
                    self.state = CommState.INIT
                    self.events.update("curr_state", CommState.INIT)

    def join(self, timeout: float | None = None) -> None:
        self.running = False
        return super().join(timeout)
    
    def send(self):
        pass
    
    def crc16(self, msg: list[int]):
        # Assuming that all items in msg can fit in byte
        # http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
        r: int = 0
        for b in msg:
            r ^= b << 8
            for _ in range(8):
                if r & 0x8000:
                    r = ((r << 1) ^ self.crc16_poly) & 0xFFFF
                else:
                    r <<= 1
        return [(r & 0xFF00) >> 8, r & 0xFF]

    def initiate_handshake_msg(self):
        msg = [START_BYTE, HANDSHAKE_BYTE, 0x01, 0x00]
        msg += self.crc16(msg)
        msg.append(END_BYTE)
        return msg
    
    def heartbeat_msg(self):
        msg = [START_BYTE, HEARTBEAT_BYTE, 0x01, 0x00]
        msg += self.crc16(msg)
        msg.append(END_BYTE)
        return msg

    def handle_handshake(self) -> bool:
        res = spidev.xfer2(self.handshake_msg)

        # TODO see if this level of checking is required
        if res[0] != START_BYTE or res[-1] != END_BYTE or res[1] != HANDSHAKE_BYTE or res[2] != 1 or sum(self.crc16(res[:-1])):
            pass # TODO resend logic
        
        # TODO more thorough interface check
        if res[3] != 0: # TODO maybe I should find some way to sync configs
            pass # TODO work out what happens when the interface numbers don't match our config
            
        return res[3] == 0

    def handle_heartbeat(self):
        res = spidev.xfer2(self.heartbeat_msg)

        # TODO see if this level of checking is required
        # If heartbeat response fails, then return for now since the controller at least responded
        if res[0] != START_BYTE or res[-1] != END_BYTE or res[1] != HEARTBEAT_BYTE or res[2] != 1 or sum(self.crc16(res[:-1])):
            self.heartbeat_fails += 1 # TODO decide failure behavior if heartbeat fails exceeds some number
            return
        
        self.heartbeat_fails = 0

        if res[3] != 0:
            pass # TODO maybe implement error codes here
    
    def handle_command(self):
        req = self.delta.items_and_clear()
        for k, v in req.items(): # TODO probably have to do data type validation
            msg = [START_BYTE, COMMAND_BYTE, 2, k, v]
            msg += self.crc16(msg)
            msg.append(END_BYTE)
            res = spidev.xfer2(msg) # TODO see if we need to check response

        

# TODO Initial handshake (testing reliability of comms, getting hardware configuration)
# 8 bytes at a time?

# MASTER
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), padding (1), CHECKSUM (2), END BYTE (1)

# SLAVE
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), NUMBER OF INTERFACE OPTIONS (1), CHECKSUM (2), END BYTE (1)

# TODO heartbeat signal?

# MASTER
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), CHECKSUM (2), END BYTE (1)

# SLAVE
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), ERROR CODE (1), CHECKSUM (2), END BYTE (1)

# TODO command

# For actuator changed:
# MASTER
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), RELEVANT DATA (x), CHECKSUM (2), END BYTE (1)

# SLAVE
# START BYTE (1), MESSAGE TYPE (1), CONTENT LENGTH (1), GENERIC ACK (x), CHECKSUM (2), END BYTE (1)