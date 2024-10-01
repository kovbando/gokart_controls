import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32  # Import Int32 for count topics and Float32 for mps topics
import serial
import threading

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # Declare parameters with default values
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('multiplier', 4.25)

        # Get parameters from the launch file or use default values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.multiplier = self.get_parameter('multiplier').get_parameter_value().double_value

        # Set up the serial connection to your device
        self.serial_port = serial.Serial(serial_port, baudrate, timeout=None)  # Blocking read with no timeout
        self.get_logger().info(f"Connected to serial port {serial_port} with baudrate {baudrate}")

        # Create publishers for the two integer topics (countLeft and countRight)
        self.publisher_left = self.create_publisher(Int32, 'countLeft', 10)
        self.publisher_right = self.create_publisher(Int32, 'countRight', 10)

        # Create publishers for the two float topics (mpsLeft and mpsRight)
        self.publisher_mps_left = self.create_publisher(Float32, 'mpsLeft', 10)
        self.publisher_mps_right = self.create_publisher(Float32, 'mpsRight', 10)

        # Start the serial reading in a separate thread
        self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.serial_thread.start()

    def is_valid_number(self, string):
        """ Helper function to check if a string is a valid integer """
        try:
            int(string)
            return True
        except ValueError:
            return False

    def read_serial_data(self):
        """ Continuously reads data from the serial port in a blocking manner and publishes to topics """
        while rclpy.ok():  # Ensure the node is active
            try:
                # Read data from the serial port
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Split the line into two values using space as the delimiter
                numbers = line.split(' ')

                # Check if we received exactly two values and if both are valid integers
                if len(numbers) == 2 and self.is_valid_number(numbers[0]) and self.is_valid_number(numbers[1]):
                    count_left = int(numbers[0])  # Parse the first number as an integer
                    count_right = int(numbers[1])  # Parse the second number as an integer

                    # Publish countLeft and countRight topics
                    msg_left = Int32()
                    msg_left.data = count_left
                    self.publisher_left.publish(msg_left)
                    self.get_logger().info(f"Published to countLeft: {count_left}")

                    msg_right = Int32()
                    msg_right.data = count_right
                    self.publisher_right.publish(msg_right)
                    self.get_logger().info(f"Published to countRight: {count_right}")

                    # Publish mpsLeft and mpsRight (multiplying by the configurable multiplier)
                    mps_left = count_left * self.multiplier
                    mps_right = count_right * self.multiplier

                    msg_mps_left = Float32()
                    msg_mps_left.data = mps_left
                    self.publisher_mps_left.publish(msg_mps_left)
                    self.get_logger().info(f"Published to mpsLeft: {mps_left}")

                    msg_mps_right = Float32()
                    msg_mps_right.data = mps_right
                    self.publisher_mps_right.publish(msg_mps_right)
                    self.get_logger().info(f"Published to mpsRight: {mps_right}")

                else:
                    self.get_logger().warn(f"Received non-numeric or invalid data: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    
    try:
        rclpy.spin(node)  # Spin the node while the serial thread runs in the background
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
