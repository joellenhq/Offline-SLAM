import multiprocessing as mp

__author__ = "Joanna Koszyk"
__contact__ = "jkoszyk@agh.edu.pl"
__copyright__ = "Copyright 2023, AGH"
__date__ = "2023/01/09"
__email__ = "jkoszyk@agh.edu.pl"
__version__ = "1.0.0"

'''This file contains a class that provides connection to ROS environment. Connection status with ROS and LiDAR 
are acquired.'''


class RosHandler(mp.Process):
    def __init__(self):
        super().__init__()
        self.roslibpy_import = False
        try:
            # import roslibpy library
            import roslibpy
            self.roslibpy_import = True
        except ImportError:
            print("roslibpy library is missing")
            return
        # ROS master connection data
        self.host = '192.168.0.100'
        self.port = 11312
        self.lidar_status = False

    # unpack the ROS message to boolean variable
    def get_lidar_status(self, message):
        self.lidar_status = message['data']

    def run(self, run_event, ros_connection_status, lidar_status):
        if self.roslibpy_import:
            print("trying to connect to ROS")
            ros_listeners_initialized = False
            try:
                # connect to ROS
                ros_client = roslibpy.Ros(host=self.host, port=self.port)
                ros_client.run()
                # create listener to the 'lidar_status' topic
                listener = roslibpy.Topic(ros_client, '/lidar_status', 'std_msgs/Bool')
                print("connection established")
                # subscribe to 'lidar_status' topic using the callback function to catch the message
                listener.subscribe(self.save_lidar_status)
                # save information on the successful connection
                ros_connection_status.value = True
                ros_listeners_initialized = True

            except roslibpy.core.RosTimeoutError:
                # save information on the faulty connection
                ros_connection_status.value = False
                ros_listeners_initialized = False

            while run_event.is_set():
                # check ros connection status
                ros_connection_status.value = ros_client.is_connected
                # if connections is not established - try connecting again
                if ros_client.is_connected:

                    lidar_status.value = self.lidar_status
                    if not ros_listeners_initialized:
                        listener = roslibpy.Topic(ros_client, '/lidar_status', 'std_msgs/Bool')
                        listener.subscribe(self.save_lidar_status)
                        ros_listeners_initialized = True
                    time.sleep(0.5)
                else:
                    lidar_status.value = False
            # terminate connection with ROS after closing the application
            if ros_client.is_connected:
                ros_client.terminate()