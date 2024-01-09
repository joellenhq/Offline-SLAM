import multiprocessing as mp
import sys
from multiprocessing import Value, Manager
from slam import Slam
from gui_handler import Receiver
from ros_handler import RosHandler


def main():

    run_event = mp.Event()
    run_event.set()

    gui = Receiver()
    ros = RosHandler()
    calc = Slam()

    calculation_trigger = Value('i', 0)
    rotation_option = Value('i', 0)
    translation_option = Value('i', 0)
    calculations_progress = Value('i', 0)
    ros_connection_status = Value('i', 0)
    lidar_status = Value('i', 0)

    x_limit = Value('i', 0)
    y_limit = Value('i', 0)
    z_limit = Value('i', 0)

    manager = Manager()
    data_path = manager.dict()
    data_path['input'] = 'E:\\hough-04-12\\examples\\corridor'
    data_path['output'] = 'E:\\hough-04-12\\examples\\corridor_ht5'

    pose = manager.dict()
    pose['positions'] = []

    p1 = mp.Process(name="gui", target=gui.run, args=[run_event, calculation_trigger, rotation_option,
                                                      translation_option, calculations_progress, ros_connection_status,
                                                      lidar_status, data_path, pose, x_limit, y_limit, z_limit])
    p2 = mp.Process(name="ros_connection", target=ros.run, args=[run_event, ros_connection_status, lidar_status])
    p3 = mp.Process(name="calculations", target=calc.run, args=[run_event, calculation_trigger, rotation_option,
                                                                translation_option, calculations_progress, data_path,
                                                                pose, x_limit, y_limit, z_limit])

    p1.start()
    p2.start()
    p3.start()

    p1.join()
    p2.join()
    p3.join()

    p1.kill()
    p2.kill()
    p3.kill()

    sys.exit()


if __name__ == "__main__":
    main()
