import multiprocessing as mp
import sys
from PyQt5.QtWidgets import QApplication
from gui import Gui

__author__ = "Joanna Koszyk"
__contact__ = "jkoszyk@agh.edu.pl"
__copyright__ = "Copyright 2023, AGH"
__date__ = "2023/01/09"
__email__ = "jkoszyk@agh.edu.pl"
__version__ = "1.0.0"

'''This file contains a class that starts GUI and saves and forwards GUI values that can be later used by 
another process'''


class Receiver(mp.Process):

    def __init__(self):
        super().__init__()

    def run(self, run_event, calculation_trigger, rotation_option, translation_option, calculations_progress,
            ros_connection_status, lidar_status, data_path, pose, icp_error, icp_max_iter):
        app = QApplication(sys.argv)
        # initialize GUI
        self.p = Gui()
        # save previous state of variable that stores clicking calculations button from GUI
        previous_calculation_trigger_value = False
        # run as long as the run_event is set
        while run_event.is_set():
            # if application closes - reset run event in order to finish all running processes
            if self.p.exit:
                run_event.clear()
            # store information if rotation option is checked
            rotation_option.value = self.p.rotateCheckBx.isChecked()
            # store information if translation option is checked
            translation_option.value = self.p.translationCheckBx.isChecked()
            # store information if running calculations were clicked
            calculation_trigger.value = self.p.calculation_trigger

            # get information on ICP settings
            icp_error.value, icp_max_iter.value = self.p.get_icp_values()

            # check if state of running calculation changed
            if calculation_trigger.value != previous_calculation_trigger_value:
                # get input and output path path
                data_path['input'] = self.p.default_path1
                data_path['output'] = self.p.default_path2
                # save previous state of calculation_trigger variable
                previous_calculation_trigger_value = calculation_trigger.value
            # if calculations started, reset calculation_trigger value
            if calculations_progress.value > 0:
                self.p.calculation_trigger = False
            # reset progress when calculations finished
            if calculations_progress.value > 99:
                calculations_progress.value = 0
            self.p.progress = calculations_progress.value
            # get ROS connection status and LiDAR status to GUI
            self.p.ros_status = ros_connection_status.value
            self.p.lidar_status = lidar_status.value
            # update connection Widgets on GUI
            self.p.connection_check()
            self.p.update_progress_control()
            # print(pose)
            self.p.poses = pose['positions']
            # update GUI
            self.p.update_image()
