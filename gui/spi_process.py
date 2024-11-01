from multiprocessing import Process, Queue
import spidev
from queue import Empty
from typing import Iterable, Mapping, Callable, Any

from messages import MESSAGES

class SpiProcess(Process):
    def __init__(
        self,
        q: Queue,
        response_q: Queue,
        init_e,
        init_failed,
        spi_bus,
        chip_num=0,
        timeout=10,
        group: None = None,
        target: Callable[..., object] | None = None,
        name: str | None = None,
        args: Iterable[Any] = ...,
        kwargs: Mapping[str, Any] = ...,
        *,
        daemon: bool | None = None,
    ) -> None:
        super().__init__(group, target, name, args, kwargs, daemon=daemon)
        self.q = q
        self.response_q = response_q
        self.init_e = init_e
        self.init_failed = init_failed
        self.timeout = timeout
        self.running = True
        self.spi_bus = spi_bus
        self.chip_num = chip_num

    def run(self):
        spi = spidev.SpiDev()
        try:
            spi.open(self.spi_bus, self.chip_num)
            spi.max_speed_hz = 100000
            spi.mode = 0
            self.init_e.set()  # trigger event if SPI didn't fail to setup
            while self.running:
                msg = self.q.get(timeout=self.timeout)
                if msg == MESSAGES["hb"]:  # TODO decide on heartbeat value
                    continue
                res = spi.xfer2(msg, spi.max_speed_hz, 100) # TODO see if delay works
                self.response_q.put(res)
                
        except FileNotFoundError:
            print("SPI Device not found")
        except PermissionError:
            print("Permission denied")
        except Empty:
            print(f"No heartbeat or messages received within {self.timeout} seconds")
        except TimeoutError:
            print("Timeout while sending SPI message")
        except Exception as e:
            print(f"General Exception: {e}")
        finally:
            spi.close()
            with self.init_failed.get_lock():
                self.init_failed.value = 1  # communicates that SPI has failed or exited
            self.init_e.set()  # TODO see if setting this every time breaks something

    def join(self, timeout: float | None = None) -> None:
        self.running = False
        return super().join(timeout)
