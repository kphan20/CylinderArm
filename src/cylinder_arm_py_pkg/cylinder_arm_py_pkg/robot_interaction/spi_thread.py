from threading import Thread
from queue import Queue, Empty
from typing import Callable, Iterable, Mapping, Any, Dict

from rclpy import ok
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
import spidev

from cylinder_arm_py_pkg.robot_interaction.messages import MESSAGES
from cylinder_arm_interfaces.msg import SPISend

class SPIHandler(Node):
    def __init__(self, spi_bus, q: Queue, sub_topic: str, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(f"SPI_RESPONSE{spi_bus}", context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)
        self.get_logger().info(f"{self.get_name()} node started")
        self.pub_map: Dict[str, Publisher] = dict()
        self.notification: Publisher = self.create_publisher(SPISend, f"spi{spi_bus}/notif", 10)
        self.send_sub = self.create_subscription(SPISend, sub_topic, self.send_callback, 10) # TODO look at QOS
        self.sub_topic = sub_topic
        self.q = q

    def send_message(self, response, topic: str):
        msg = SPISend()
        msg.message = response

        publisher = self.pub_map.get(topic, self.create_publisher(SPISend, topic, 10))
        publisher.publish(msg)
        self.pub_map[topic] = publisher
    
    def send_notification(self, notification: SPISend):
        notification.return_topic = self.sub_topic
        self.notification.publish(notification)

    def send_callback(self, msg):
        self.q.put(msg)

class SPIThread(Thread):
    def __init__(
        self,
        spi_bus,
        no_hw,
        chip_num=0,
        timeout=10,
        group: None = None,
        target: Callable[..., object] | None = None,
        name: str | None = None,
        daemon: bool | None = None,
        *args: Iterable[Any],
        **kwargs: Mapping[str, Any],
    ) -> None:
        super().__init__(group, target, name, args, kwargs, daemon=daemon)
        self.q = Queue()
        self.timeout = timeout
        self.running = True
        self.spi_bus = spi_bus
        self.chip_num = chip_num
        self.executor = SingleThreadedExecutor() # rclpy.executors.MultiThreadedExecutor()
        self.ros_node = SPIHandler(spi_bus, self.q, f"spi{spi_bus}/send")
        self.hw_available = not no_hw

    def send_notification(self, message_code: int):
        if ok():
            notif = SPISend()
            notif.spi_bus = self.spi_bus
            notif.status_message = message_code
            self.ros_node.send_notification(notif)

    def custom_spin(self):
        try:
            while self.running and self.executor._context.ok() and not self.executor._is_shutdown:
                self.executor.spin_once(0.2)
        except Exception as e:
            pass

    def run(self):
        spi = spidev.SpiDev() if self.hw_available else None
        
        try:
            if spi is not None:
                spi.open(self.spi_bus, self.chip_num)
                spi.max_speed_hz = 100000
                spi.mode = 0

            self.executor.add_node(self.ros_node)
            spin_thread = Thread(target=self.custom_spin)
            spin_thread.start()

            # self.init_e.set()  # trigger event if SPI didn't fail to setup
            while self.running and ok():            
                msg: SPISend = self.q.get(timeout=self.timeout) # TODO see how message bytes are sent and received
                msg_data, topic, spi_bus = msg.message, msg.return_topic, msg.spi_bus
                if spi_bus != self.spi_bus:
                    continue # TODO return wrong spi bus message
                if status_msg == MESSAGES["hb"]:
                    self.send_notification(MESSAGES["ready"]) # inform manager things are okay
                    continue
                elif status_msg == MESSAGES["terminate"]:
                    break
                if spi is not None:
                    res = spi.xfer2(msg_data, spi.max_speed_hz, 100) # TODO see if delay works
                    self.ros_node.send_message(res, topic)
                
        except FileNotFoundError:
            self.ros_node.get_logger().error("SPI Device not found")
        except PermissionError:
            self.ros_node.get_logger().error("Permission denied")
        except Empty:
            self.ros_node.get_logger().error(f"No heartbeat or messages received within {self.timeout} seconds")
        except TimeoutError:
            self.ros_node.get_logger().error("Timeout while waiting for SPI message")
        except KeyboardInterrupt:
            self.ros_node.get_logger().error("Keyboard interrupt detected. Exiting")
        except ExternalShutdownException:
            self.ros_node.get_logger().info("Keyboard interrupt received. Exiting")
        except Exception as e:
            self.ros_node.get_logger().error(f"General Exception: {e}")
        finally:
            if spi is not None:
                spi.close()
            self.send_notification(MESSAGES["unavailable"])
            self.running = False
            spin_thread.join()
            self.ros_node.destroy_node()
            self.executor.shutdown()
            self.ros_node.get_logger().info(f"SPI Process on bus {self.spi_bus} closed")

    def join(self, timeout: float | None = None) -> None:
        self.running = False
        return super().join(timeout)