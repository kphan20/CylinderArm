from rclpy.node import Node
from threading import Event

from cylinder_arm_interfaces.msg import SPISend
from cylinder_arm_interfaces.srv import HardwareStartReq
from .messages import MESSAGES

class TouchScreenNode(Node):
    def __init__(self, spi_bus, init_event: Event, spi_failed, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__('touchscreen_node', context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)
        self.send_sub = self.create_subscription(SPISend, f"spi{spi_bus}/notif", self.notification_callback, 10)
        self.init_event = init_event
        self.spi_failed = spi_failed

        self.hardware_client = self.create_client(HardwareStartReq, 'initiate_hardware')
        self.hardware_req = HardwareStartReq.Request()
        self.hardware_req.spi_bus = spi_bus

    # sends hardware request - manager will send ready message when hardware is setup
    def start_hardware(self):
        if self.hardware_client.service_is_ready():
            return self.hardware_client.call_async(self.hardware_req)
        else:
            self.get_logger().info('Hardware service not available, waiting...')

    def notification_callback(self, msg: SPISend):
        # upon hardware beginning to communicate, notify party trying to initiate contact
        if msg.message[0] == MESSAGES['ready']:
            self.init_event.set()
        
        # set failure upon hardware failure
        elif msg.message[0] == MESSAGES['unavailable']:
            self.spi_failed.set(1) # communicates that SPI has failed or exited

    def send_message(self, msg):
        pass
