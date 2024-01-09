import pyqtgraph as pg
import os
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pandas as pd
import open3d as o3d
import fnmatch
import pyqtgraph.opengl as gl
from PyQt5.QtWidgets import QMessageBox, QFileDialog

__author__ = "Joanna Koszyk"
__contact__ = "jkoszyk@agh.edu.pl"
__copyright__ = "Copyright 2023, AGH"
__date__ = "2023/01/09"
__email__ = "jkoszyk@agh.edu.pl"
__version__ = "1.0.0"


'''
This file contains GUI created with the use of PyQt library. It creates widgets to link the point cloud input and 
output directories. Additionally, 3D and 2D view is available. The application allows to start the calculations 
with rotation and translation. After calculations are done, pose estimation can be seen at the Localization panel. 
'''


class Gui(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(Gui, self).__init__(parent)
        pg.setConfigOptions(antialias=True)

        # 3D View Widget
        self.glv_win1 = gl.GLViewWidget()

        # create 3D scatter plot to show input data
        self.color = (1.0, 1.0, 1.0, 0.5)
        self.sp1 = gl.GLScatterPlotItem(pos=[0, 0, 0], size=0.1, color=self.color, pxMode=False)
        self.glv_win1.addItem(self.sp1)

        # create 2D scatter plot to show output data
        # self.data = pcld1
        self.view = pg.GraphicsLayoutWidget()
        self.w1 = self.view.addPlot()
        self.scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))

        # self.scatter.setData(self.data[:, 1], self.data[:, 0])
        self.w1.addItem(self.scatter)

        # create labels for folders
        self.folderLabel1 = QtGui.QLabel()
        self.folderLabel2 = QtGui.QLabel()

        # create font for label Widgets
        font = QtGui.QFont()
        font.setPointSize(10)

        # create buttons for path selection
        self.choosePathBtn1 = QtGui.QPushButton()
        self.choosePathBtn2 = QtGui.QPushButton()
        # create button to close the application
        self.exitBtn = QtGui.QPushButton()

        # create labels to show point cloud directories on GUI
        self.path1 = QtGui.QLabel()
        self.path1.setEnabled(False)

        font = QtGui.QFont()
        font.setPointSize(10)
        self.path1.setFont(font)
        self.path2 = QtGui.QLabel()
        self.path2.setEnabled(False)
        self.path2.setFont(font)

        font2 = QtGui.QFont('Times')
        font2.setPointSize(11)
        font2.setBold(True)

        self.pathLabel1 = QtGui.QLabel()
        self.pathLabel1.setFont(font)
        self.pathLabel2 = QtGui.QLabel()
        self.pathLabel2.setFont(font)

        self.default_path1 = 'E:\\hough-04-12\\examples\\corridor'
        self.default_path2 = 'E:\\hough-04-12\\examples\\corridor_ht5'
        self.path1.setText(self.default_path1)
        self.path2.setText(self.default_path2)

        # create buttons to open Explorer from GUI
        self.openExplorerBtn1 = QtGui.QPushButton()
        self.openExplorerBtn2 = QtGui.QPushButton()

        # put Widgets in a groupbox
        self.groupbox1 = QtGui.QGroupBox()
        self.groupbox1.setCheckable(False)
        self.vbox1 = QtGui.QHBoxLayout()
        self.groupbox1.setLayout(self.vbox1)
        self.vbox1.addWidget(self.path1)
        self.vbox1.addWidget(self.openExplorerBtn1)

        # put Widgets in a groupbox
        self.groupbox2 = QtGui.QGroupBox()
        self.groupbox2.setCheckable(False)
        self.vbox2 = QtGui.QHBoxLayout()
        self.groupbox2.setLayout(self.vbox2)
        self.vbox2.addWidget(self.path2)
        self.vbox2.addWidget(self.openExplorerBtn2)

        # Offline SLAM Widgets
        self.groupbox3 = QtGui.QGroupBox()
        self.groupbox3.setCheckable(False)
        self.vbox3 = QtGui.QVBoxLayout()
        self.groupbox3.setLayout(self.vbox3)
        self.mappingLabel = QtGui.QLabel()
        self.mappingLabel.setFont(font2)
        # create button to run calculations
        self.mappingBtn = QtGui.QPushButton()

        # create controls to set the settings for ICP method
        # acceptable error in mam alignment
        self.icp_error_label = QtGui.QLabel()
        self.icp_error_spinbox = QtGui.QSpinBox()
        self.icp_error_spinbox.setMaximum(2000)
        self.icp_error_spinbox.setMinimum(0)
        self.icp_error_spinbox.setSingleStep(10)
        self.icp_error_spinbox.setValue(50)

        # maximum number of iterations
        self.icp_max_iter_label = QtGui.QLabel()
        self.icp_max_iter_spinbox = QtGui.QSpinBox()
        self.icp_max_iter_spinbox.setMaximum(2000)
        self.icp_max_iter_spinbox.setMinimum(0)
        self.icp_max_iter_spinbox.setSingleStep(20)
        self.icp_max_iter_spinbox.setValue(70)

        self.rotateCheckBx = QtGui.QCheckBox("Rotation")
        self.rotateCheckBx.setCheckable(True)
        self.translationCheckBx = QtGui.QCheckBox("Translation")
        self.translationCheckBx.setCheckable(True)

        self.localizationLabel = QtGui.QLabel()
        self.localizationLabel.setFont(font2)
        self.poseLabel = QtGui.QLabel()

        # create labels to show robot pose on GUI
        self.x_pose_label = QtGui.QLabel()
        self.y_pose_label = QtGui.QLabel()
        self.z_pose_label = QtGui.QLabel()
        self.yaw_pose_label = QtGui.QLabel()
        self.x_pose_value = QtGui.QLabel()
        self.y_pose_value = QtGui.QLabel()
        self.z_pose_value = QtGui.QLabel()
        self.yaw_pose_value = QtGui.QLabel()

        self.vbox3.addWidget(self.mappingLabel)
        self.vbox3.addWidget(self.icp_error_label)
        self.vbox3.addWidget(self.icp_error_spinbox)
        self.vbox3.addWidget(self.icp_max_iter_label)
        self.vbox3.addWidget(self.icp_max_iter_spinbox)
        self.vbox3.addWidget(self.rotateCheckBx)
        self.vbox3.addWidget(self.translationCheckBx)
        self.vbox3.addWidget(self.mappingBtn)
        self.vbox3.addWidget(self.localizationLabel)
        self.vbox3.addWidget(self.poseLabel)

        self.groupbox4 = QtGui.QGroupBox()
        self.groupbox4.setCheckable(False)
        self.vbox4 = QtGui.QHBoxLayout()
        self.groupbox4.setLayout(self.vbox4)
        self.vbox4.addWidget(self.x_pose_label)
        self.vbox4.addWidget(self.x_pose_value)

        self.groupbox5 = QtGui.QGroupBox()
        self.groupbox5.setCheckable(False)
        self.vbox5 = QtGui.QHBoxLayout()
        self.groupbox5.setLayout(self.vbox5)
        self.vbox5.addWidget(self.y_pose_label)
        self.vbox5.addWidget(self.y_pose_value)

        self.groupbox6 = QtGui.QGroupBox()
        self.groupbox6.setCheckable(False)
        self.vbox6 = QtGui.QHBoxLayout()
        self.groupbox6.setLayout(self.vbox6)
        self.vbox6.addWidget(self.z_pose_label)
        self.vbox6.addWidget(self.z_pose_value)

        self.groupbox7 = QtGui.QGroupBox()
        self.groupbox7.setCheckable(False)
        self.vbox7 = QtGui.QHBoxLayout()
        self.groupbox7.setLayout(self.vbox7)
        self.vbox7.addWidget(self.yaw_pose_label)
        self.vbox7.addWidget(self.yaw_pose_value)

        self.vbox3.addWidget(self.groupbox4)
        self.vbox3.addWidget(self.groupbox5)
        self.vbox3.addWidget(self.groupbox6)
        self.vbox3.addWidget(self.groupbox7)

        # create labels that indicate which point cloud is shown
        self.source_point_cloud_label = QtGui.QLabel()
        self.output_point_cloud_label = QtGui.QLabel()
        self.source_point_cloud_label.setFont(font2)
        self.output_point_cloud_label.setFont(font2)
        # create button to show the next point cloud
        self.next_cloud_btn = QtGui.QPushButton()

        # control to show ROS status
        self.ros_status_btn = QtGui.QPushButton()
        # control to show LiDAR status
        self.lidar_status_btn = QtGui.QPushButton()
        # control to shown calculations progress
        self.calculations_status_btn = QtGui.QPushButton()

        self.ros_status_btn.setEnabled(False)
        self.lidar_status_btn.setEnabled(False)
        self.calculations_status_btn.setEnabled(False)

        self.groupbox8 = QtGui.QGroupBox()
        self.groupbox8.setCheckable(False)
        self.vbox8 = QtGui.QVBoxLayout()
        self.groupbox8.setLayout(self.vbox8)
        self.vbox8.addWidget(self.ros_status_btn)
        self.vbox8.addWidget(self.lidar_status_btn)
        self.vbox8.addWidget(self.calculations_status_btn)

        # create main layout
        self.layout = pg.LayoutWidget()
        self.layout.addWidget(self.pathLabel1, 0, 0)
        self.layout.addWidget(self.groupbox1, 0, 1, colspan=2)
        self.layout.addWidget(self.choosePathBtn1, 0, 3)
        self.layout.addWidget(self.pathLabel2, 1, 0)
        self.layout.addWidget(self.groupbox2, 1, 1, colspan=2)
        self.layout.addWidget(self.choosePathBtn2, 1, 3)
        self.layout.addWidget(self.groupbox3, 2, 3, rowspan=6)
        self.layout.addWidget(self.source_point_cloud_label, 2, 0)
        self.layout.addWidget(self.glv_win1, 3, 0, colspan=3, rowspan=5)
        self.layout.addWidget(self.output_point_cloud_label, 8, 0)
        self.layout.addWidget(self.view, 9, 0, colspan=3, rowspan=2)
        self.layout.addWidget(self.next_cloud_btn, 11, 0, colspan=3, rowspan=1)
        self.layout.addWidget(self.groupbox8, 9, 3, rowspan=2)
        self.layout.addWidget(self.exitBtn, row=12, col=0, colspan=4)
        self.layout.resize(800, 800)
        # unable default close button
        self.layout.setWindowFlag(QtCore.Qt.WindowCloseButtonHint, False)
        # show layout
        self.layout.show()

        # connect buttons to functions
        self.exitBtn.clicked.connect(self.close_window)
        self.choosePathBtn1.clicked.connect(lambda: self.choose_path(1))
        self.choosePathBtn2.clicked.connect(lambda: self.choose_path(2))
        self.openExplorerBtn1.clicked.connect(lambda: self.open_explorer(1))
        self.openExplorerBtn2.clicked.connect(lambda: self.open_explorer(2))
        self.mappingBtn.clicked.connect(self.calculate)
        self.next_cloud_btn.clicked.connect(self.load_next_pcd)

        # variable to store the number of currently shown point cloud
        self.point_cloud_no = 0
        # variable to store triggering the calculations
        self.calculation_trigger = False
        # ROS connection status
        self.ros_status = False
        # LiDAR connection status
        self.lidar_status = False
        # poses for each point cloud
        self.poses = np.array(np.zeros((20, 4)))
        self.progress = 0

        _translate = QtCore.QCoreApplication.translate

        self.choosePathBtn1.setText(_translate("MainWindow", "..."))
        self.choosePathBtn2.setText(_translate("MainWindow", "..."))
        self.openExplorerBtn1.setText(_translate("MainWindow", "Open folder"))
        self.openExplorerBtn2.setText(_translate("MainWindow", "Open folder"))
        self.pathLabel1.setText(_translate("MainWindow", "Data:"))
        self.pathLabel2.setText(_translate("MainWindow", "Output folder:"))
        self.mappingLabel.setText(_translate("MainWindow", "Mapping"))
        self.icp_error_label.setText(_translate("MainWindow", "ICP error: [mm]"))
        self.icp_max_iter_label.setText(_translate("MainWindow", "ICP error: [mm]"))
        self.mappingBtn.setText(_translate("MainWindow", "Calculate"))
        self.localizationLabel.setText(_translate("MainWindow", "Localization"))
        self.poseLabel.setText(_translate("MainWindow", "Pose"))
        self.x_pose_label.setText(_translate("MainWindow", "X: "))
        self.y_pose_label.setText(_translate("MainWindow", "Y: "))
        self.z_pose_label.setText(_translate("MainWindow", "Z: "))
        self.yaw_pose_label.setText(_translate("MainWindow", "Yaw: "))
        self.x_pose_value.setText(_translate("MainWindow", "0.0"))
        self.y_pose_value.setText(_translate("MainWindow", "0.0"))
        self.z_pose_value.setText(_translate("MainWindow", "0.0"))
        self.yaw_pose_value.setText(_translate("MainWindow", "0.0"))
        self.source_point_cloud_label.setText(_translate("MainWindow", "Source point cloud"))
        self.output_point_cloud_label.setText(_translate("MainWindow", "Output point cloud"))
        self.next_cloud_btn.setText(_translate("MainWindow", "Next point cloud"))
        self.ros_status_btn.setText(_translate("MainWindow", "ROS"))
        self.lidar_status_btn.setText(_translate("MainWindow", "LiDAR"))
        self.calculations_status_btn.setText(_translate("MainWindow", "0%"))
        self.exitBtn.setText(_translate("MainWindow", "Exit"))

        self.exit = False

    # set boolean value to close the application
    def close_window(self):
        self.exit = True

    # update GUI
    def update_image(self):
        QtWidgets.QApplication.processEvents()

    # choose input or output path depending on the number
    def choose_path(self, number):
        # open window to choose path
        new_path = str(QFileDialog.getExistingDirectory(self.layout, "Select Directory"))

        var_name = "default_path"+str(number)

        if len(new_path) >= 4:
            setattr(self, var_name, new_path)
        # print(self.default_path)

        setattr(self, var_name, eval(("self."+var_name)).replace("/", "\\"))
        path = eval(("self."+var_name)).split("\\")
        # print(path)
        if len(path) > 3:
            label_path = path[0] + '\\' + path[1] + '\\...\\' + path[-1]
        else:
            label_path = eval(("self."+var_name))
        # print(label_path)
        if number == 1:
            self.path1.setText(label_path)
        else:
            self.path2.setText(label_path)

    # open Explorer in the path defined in GUI
    def open_explorer(self, number):
        var_name = "self.default_path"+str(number)
        path = eval(var_name)
        os.startfile(path)

    # set the flag to trigger calculations
    def calculate(self):
        # check if folder is not empty
        no_files = 0
        for root, dirs, files in os.walk(self.default_path1):
            for extension in ('*.csv', '*.pcd', '*.ply'):
                for _ in fnmatch.filter(files, extension):
                    no_files += 1
        print(no_files)
        if no_files > 0:
            self.calculation_trigger = True
        else:
            self.show_message_box()

    def show_message_box(self):
        QMessageBox.about(self.layout, "Empty folder", "There is no files in the input data folder")

    # load the next point cloud on the 2d and 3d scatter plot
    def load_next_pcd(self):
        # get path to csv files in the chosen folder
        input_files_list = fnmatch.filter(os.listdir(self.default_path1), "*.csv")
        output_files_list = fnmatch.filter(os.listdir(self.default_path2), "*.csv")
        if len(input_files_list) == 0:
            input_files_list = fnmatch.filter(os.listdir(self.default_path1), "*.pcd")

        # choose the next point cloud in the folder
        if (len(input_files_list) - 1) > self.point_cloud_no:
            self.point_cloud_no += 1
        else:
            self.point_cloud_no = 0
        # get full path to a point cloud file
        input_file = os.path.join(self.default_path1, input_files_list[self.point_cloud_no])
        # update 3D scatter plot
        self.update_3d_plot(input_file)
        # check if the result matching the input data exists
        if (len(output_files_list) - 1) < self.point_cloud_no:
            print("first do the calculations")
        else:
            # get full path to a point cloud file
            output_file = os.path.join(self.default_path2, output_files_list[self.point_cloud_no])
            # update 2d scatter plot
            self.update_2d_plot(output_file)
            # update pose labels
            self.read_pose()

    # update 3d scatter plot - show input data (csv point cloud) on GUI
    def update_3d_plot(self, pcd_path):
        data = Gui.pcd_to_array(pcd_path)
        self.source_point_cloud_label.setText(("Source point cloud  "+str(pcd_path)))
        self.sp1.setData(pos=data[:, 0:3])

    # update 2d scatter plot - show output data (csv point cloud) on GUI
    def update_2d_plot(self, pcd_path):
        data = Gui.pcd_to_array(pcd_path)
        self.output_point_cloud_label.setText(("Output point cloud  " + str(pcd_path)))
        self.scatter.setData(data[:, 0], data[:, 1])
        self.scatter.show()

    # update connection controls Widgets on GUI
    def connection_check(self):
        if self.lidar_status:
            self.lidar_status_btn.setStyleSheet("background-color : green; color: black")
        else:
            self.lidar_status_btn.setStyleSheet("background-color : grey; color: black")
        if self.ros_status:
            self.ros_status_btn.setStyleSheet("background-color : green; color: black")
        else:
            self.ros_status_btn.setStyleSheet("background-color : grey; color: black")

    def read_pose(self):
        # print(self.poses)
        self.x_pose_value.setText(f"{self.poses[self.point_cloud_no][0]:.3f}")
        self.y_pose_value.setText(f"{self.poses[self.point_cloud_no][1]:.3f}")
        self.z_pose_value.setText(f"{self.poses[self.point_cloud_no][2]:.3f}")
        self.yaw_pose_value.setText(f"{self.poses[self.point_cloud_no][3]:.3f}")

    def update_progress_control(self):
        # change visual traits of a Widget (PushButton) that show progress of calculations
        if (self.progress > 0) and (self.progress < 100):
            self.calculations_status_btn.setText(f"In progress: {self.progress}%")
            self.calculations_status_btn.setStyleSheet("background-color : green; color: black")
        else:
            self.calculations_status_btn.setStyleSheet("background-color : grey; color: black")
            self.calculations_status_btn.setText(" ")

    # get limits from
    def get_icp_values(self):
        icp_error = self.icp_error_spinbox.value()
        icp_max_iter = self.icp_max_iter_spinbox.value()
        return icp_error, icp_max_iter

    # given the path to a point cloud, return the numpy array with points
    @staticmethod
    def pcd_to_array(pcd_path):
        # print(pcd_path[-3:])
        if pcd_path[-3:] == "csv":
            cloud0 = pd.read_csv(pcd_path, header=None)
        else:
            pcd = o3d.io.read_point_cloud(pcd_path)
            cloud0 = np.asarray(pcd.points)
        pcld1 = np.array(cloud0)
        return pcld1
