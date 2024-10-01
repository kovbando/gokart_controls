import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Change to Int32 for integer publishing
import serial
import threading

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # Set up the serial connection to your device
        self.serial_port = serial.Serial('/dev/ttyACM1', 9600, timeout=None)  # Blocking read with no timeout
        self.get_logger().info("Connected to serial port /dev/ttyACM0")

        # Create publishers for the two topics (countLeft and countRight)
        self.publisher_left = self.create_publisher(Int32, 'countLeft', 10)
        self.publisher_right = self.create_publisher(Int32, 'countRight', 10)

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

                    # Create and publish messages for each count
                    msg_left = Int32()
                    msg_left.data = count_left
                    self.publisher_left.publish(msg_left)
                    self.get_logger().info(f"Published to countLeft: {count_left}")

                    msg_right = Int32()
                    msg_right.data = count_right
                    self.publisher_right.publish(msg_right)
                    self.get_logger().info(f"Published to countRight: {count_right}")
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

