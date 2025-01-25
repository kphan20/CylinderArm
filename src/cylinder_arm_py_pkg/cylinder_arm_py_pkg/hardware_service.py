from cylinder_arm_interfaces.srv import HardwareStartReq
from cylinder_arm_interfaces.msg import SPISend
from cylinder_arm_py_pkg.robot_interaction.spi_thread import SPIThread
from cylinder_arm_py_pkg.robot_interaction.messages import MESSAGES

from rclpy import init, spin, shutdown, ok
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.executors import ExternalShutdownException
from threading import Thread
from typing import Dict, Tuple

class HardwareService(Node):
    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)

        # service for other nodes to create threads to communicate with hardware
        self.srv = self.create_service(HardwareStartReq, 'initiate_hardware', self.hardware_start_req)

        # parameter for debugging without hardware
        self.declare_parameter("NO_HARDWARE", False)

        # manages and stores all hardware threads and communication
        self.thread_dict: Dict[str, Thread] = dict()
        self.notification_subs: Dict[str, Subscription] = dict()
        self.hb_publishers: Dict[str, Publisher] = dict()

        # bool marks if heartbeat response was resceived, int is number of failures in a row
        self.hb_states: Dict[str, Tuple[bool, int]] = dict()
        self.heartbeat_interval = 1
        self.heartbeat_timer = self.create_timer(self.heartbeat_interval, self.send_heartbeats)

    def send_heartbeats(self):
        for interface, publisher in self.hb_publishers.items():
            # handle heartbeat failures
            hb_received, hb_count = self.hb_states[interface]
            hb_count = 0 if hb_received else hb_count + 1
            if hb_count > 5:
                continue # TODO restart protocol

            # prepare for next heartbeat
            self.hb_states[interface] = (False, hb_count)

            # send heartbeat
            msg = SPISend() # TODO should I cache messages?
            msg.spi_bus = int(interface[3:])
            msg.status_message = MESSAGES['hb']
            publisher.publish(msg)

    def notification_callback(self, msg):
        interface = f"spi{msg.spi_bus}"
        if msg.status_message == MESSAGES["ready"]:
            self.hb_states[interface] = (True, 0) # TODO do I need a lock?
            return
        
        # if thread returns error, clean up failed thread
        self.remove_interface(interface)

        if interface in self.notification_subs:
            self.destroy_subscription(self.notification_subs[interface])
            del self.notification_subs[interface]
        
        if interface in self.hb_publishers:
            self.destroy_publisher(self.hb_publishers[interface])
            del self.hb_publishers[interface]

    def hardware_start_req(self, req, res):
        interface_name = f"spi{req.spi_bus}"
        no_hw = self.get_parameter("NO_HARDWARE").get_parameter_value().bool_value
        # if thread has not been started yet, set up all communications
        if interface_name not in self.thread_dict:
            t = SPIThread(req.spi_bus, no_hw, daemon=True)
            t.start()
            self.thread_dict[interface_name] = t
            notif_topic = f"{interface_name}/notif"
            self.notification_subs[interface_name] = self.create_subscription(SPISend, notif_topic, self.notification_callback, 10)
            pub_topic = f"{interface_name}/send"
            self.hb_publishers[interface_name] = self.create_publisher(SPISend, pub_topic, 10)
            self.hb_states[interface_name] = (False, 0)

        res.success = True
        return res

    def remove_interface(self, interface_name):
        if interface_name not in self.thread_dict:
            return
        self.thread_dict[interface_name].join()
        del self.thread_dict[interface_name]
        del self.hb_states[interface_name]

    def cleanup(self):
        self.heartbeat_timer.cancel()
        for t in self.thread_dict.values():
            t.join()
    
    def destroy_node(self):
        self.cleanup()
        return super().destroy_node()

def main(args=None):
    init(args=args)
    hw_service = HardwareService("HW_Service")
    try:
        spin(hw_service)
    except KeyboardInterrupt:
        hw_service.get_logger().info("Keyboard interrupt received.")
    except ExternalShutdownException:
        hw_service.get_logger().info("Keyboard interrupt received.")
    except Exception as e:
        hw_service.get_logger().error(f"Error: {e}")
    finally:
        hw_service.get_logger().info("Shutting down...")
        hw_service.destroy_node()
        if ok():
            shutdown()

if __name__ == '__main__':
    main()