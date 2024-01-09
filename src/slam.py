import multiprocessing as mp
import os
import numpy as np
from glob import glob
from tqdm import tqdm
import math
import pandas as pd
import open3d as o3d
from sklearn.cluster import DBSCAN
from skimage.transform import hough_line, hough_line_peaks
import glob

__author__ = "Joanna Koszyk"
__contact__ = "jkoszyk@agh.edu.pl"
__copyright__ = "Copyright 2023, AGH"
__date__ = "2023/01/09"
__email__ = "jkoszyk@agh.edu.pl"
__version__ = "1.0.0"


class Slam(mp.Process):

    def __init__(self):
        super().__init__()
        # max Z value (ceiling)
        self.max_z = 2.0
        # min Z value (floor)
        self.min_z = 0.1
        # maximum number of points in a point clouds
        self.max_points = 10000
        # path to input data folder
        self.input_data_path = 'E:\\hough-04-12\\examples\\corridor'
        # path to output data folder
        self.output_data_path = 'E:\\hough-04-12\\examples\\corridor_ht5'
        # list with path to input data files
        self.input_data_files = []
        # list with path to output data files
        self.output_data_files = []
        # option to cut ceiling and floor
        self.cut_cloud = True
        # option to reduce third dimension
        self.flatten_cloud = True
        # option to reduce the number of points in a point cloud
        self.thin_cloud = False
        # variable to scale point cloud for rotation
        self.pcd_scalar = 5
        # list to store poses for eac point cloud
        self.pose = []
        # a placeholder for the reference point cloud
        # the first point cloud will be set as the reference point cloud
        self.reference_pcd = np.empty((20, 4), dtype='float32')

        # set constraints
        self.constraint_x = 0.5  # m
        self.constraint_y = 0.5  # m
        self.constraint_z = 0.2  # m
        self.constraint_yaw = 90  # degrees

        # acceptable error for translation (for ICP algorithm)
        self.error = 0.05

        # maximal number of tries to reach the ICP correspondence of two point clouds
        self.max_iterations = 70

    def run(self, run_event, calculation_trigger, rotation_option, translation_option, calculations_progress,
            data_path, pose,x_limit, y_limit, z_limit):
        while run_event.is_set():
            # check if calculations were triggered in GUI
            calculations = calculation_trigger.value
            if calculations:

                # empty list wit poses
                self.pose = []
                # freeze options from GUI - check if rotation and translation were chosen
                rotate = rotation_option.value
                translate = translation_option.value

                # freeze options of limits from GUI
                self.constraint_x = x_limit.value
                self.constraint_y = y_limit.value
                self.constraint_z = z_limit.value

                # print(data_path)
                self.input_data_path = data_path['input']
                self.output_data_path = data_path['output']
                # get the list of files in input data folder
                self.input_data_files = self.get_the_list_of_files(self.input_data_path)
                # self.output_data_files = self.get_the_list_of_files(self.output_data_path)
                calculations_progress.value = 1
                # perform calculations for each point cloud in the input folder
                for i, point_cloud_file in enumerate(tqdm(self.input_data_files)):
                    # check the file type and get the point cloud as a numpy array
                    extension = point_cloud_file[-3:]
                    # print(point_cloud_file)
                    # print(extension)
                    if extension == "csv":
                        pcd_arr = self.read_csv_file(point_cloud_file)
                    elif extension == "pcd":
                        pcd_arr = self.read_pcd_file(point_cloud_file)
                    elif extension == "ply":
                        pcd_arr = self.read_ply_file(point_cloud_file)
                    else:
                        print("extension does not match")
                        break

                    # if the option was chosen, cut the point cloud
                    if self.cut_cloud:
                        pcd_arr = self.cut_ceiling(pcd_arr)
                    # if the option was chosen, reduce the z dimension
                    if self.flatten_cloud:
                        pcd_arr = self.flatten_pcd(pcd_arr)
                    # if the option was chosen, reduce the number of points
                    if self.thin_cloud:
                        pcd_arr = self.thin_pcd(pcd_arr)
                    # relocate the point cloud to positive values
                    pcd_arr = self.move_point_cloud_to_positive_values(pcd_arr)

                    dx, dy, dz, theta = 0.0, 0.0, 0.0, 0.0
                    if rotate:
                        pcd_arr_copy = pcd_arr.copy()
                        theta = self.calculate_rotation(pcd_arr_copy)
                        pcd_arr = self.rotate_pcd(pcd_arr, theta)
                    if translate:
                        dx, dy = self.calculate_position_with_icp(pcd_arr, i)
                    # save pose for the point cloud
                    # print("theta:", theta)
                    self.pose.append([dx, dy, dz, math.degrees(theta)])

                    # save new point cloud (after rotation and translation)
                    file_name = point_cloud_file.split('\\')
                    # print(file_name)
                    # print(file_name[-1])
                    # print("here")
                    # print(self.output_data_path)
                    output_path = os.path.join(self.output_data_path, (file_name[-1][:-4]+"_transformed.csv"))
                    # print(output_path)
                    self.save_point_cloud(pcd_arr, output_path)

                    calculations_progress.value = int((i+1)/len(self.input_data_files)*100)
                    # print(i)
                    # print(len(self.input_data_files))
                    # print(calculations_progress.value)

                pose['positions'] = self.pose

    def get_the_list_of_files(self, path):
        files = []
        for pcd in glob.glob(os.path.join(path, "*.*")):
            files.append(pcd)
        return files

    def read_csv_file(self, csv_path):
        cloud0 = pd.read_csv(csv_path, header=None)
        return np.array(cloud0)

    def read_pcd_file(self, pcd_path):
        pcd_format = o3d.io.read_point_cloud(pcd_path)
        return np.asarray(pcd_format.points)

    def read_ply_file(self, ply_path):
        ply_format = o3d.io.read_point_cloud(ply_path)
        return np.asarray(ply_format.points)

    def cut_ceiling(self, pcd):
        if pcd.shape[1] > 2:
            pcd = pcd[np.where(pcd[:, 2] < self.max_z)]
            pcd = pcd[np.where(pcd[:, 2] > self.min_z)]
        return pcd

    def flatten_pcd(self, pcd):
        pcd = pcd[:, 0:2]
        return pcd

    # thin point cloud
    def thin_pcd(self, pcd):
        pcd_ind = np.random.choice(pcd.shape[0], self.max_points, replace=False)
        pcd = pcd[pcd_ind]
        return pcd

    def calculate_extremum_values(self, pcd):
        min_x = np.min(pcd[:, 0])
        max_x = np.max(pcd[:, 0])
        min_y = np.min(pcd[:, 1])
        max_y = np.max(pcd[:, 1])
        if pcd.shape[1] > 2:
            min_z = np.min(pcd[:, 2])
            max_z = np.max(pcd[:, 2])
            center = ((max_x-min_x)/2, (max_y-min_y)/2, (max_z-min_z)/2)
            extremum_values = (max_x, min_x, max_y, min_y, max_z, min_z)
        else:
            center = ((max_x - min_x) / 2, (max_y - min_y) / 2)
            extremum_values = (max_x, min_x, max_y, min_y)
        return center, extremum_values

    def move_point_cloud_to_positive_values(self, pcd):
        center, max_values = self.calculate_extremum_values(pcd)
        if pcd.shape[1] > 2:
            pcd[:, 2] -= max_values[5] - 1.0
        pcd[:, 0] -= max_values[3] - 1.0
        pcd[:, 1] -= max_values[1] - 1.0
        return pcd

    def segmentation(self, pcd):
        # segmentation is not based on Z value
        pcd_xy = pcd[:, 0:2]
        # maximum distance between points to be neighbours
        eps = 0.5
        # minimum number of neighbours
        min_samples = 5
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        clusters = dbscan.fit_predict(pcd_xy)
        return clusters

    # rotation of the point cloud with rotation_angle
    def rotate_pcd(self, pcd, rotation_angle):
        # create rotation matrix
        pcd_shape = pcd.shape[1]
        # print(pcd_shape)
        if pcd_shape < 3:
            z = np.zeros(pcd.shape[0])
            z = z.reshape((-1, 1))
            pcd = np.hstack((pcd, z))
        # print(pcd)
        rotation_matrix = np.array([[math.cos(rotation_angle), -math.sin(rotation_angle), 0],
                                    [math.sin(rotation_angle), math.cos(rotation_angle), 0],
                                    [0, 0, 1]])

        # shape of the point cloud
        pcd_shape1 = np.shape(pcd)
        # new point cloud with the same shape
        new_pcd = np.empty(pcd_shape1)
        # create a copy of a point cloud
        pcd_copy = pcd.copy()

        for i in range(pcd_shape1[0]):
            xyz = pcd_copy[i, :]
            pcd_coordinates = np.transpose(xyz)
            # calculate rotation
            new_values = np.dot(rotation_matrix, pcd_coordinates)
            new_pcd[i, :] = new_values
        # return point cloud after rotation
        if pcd_shape > 2:
            return new_pcd
        else:
            return new_pcd[:, 0:2]

    def translate_pcd(self, pcd, translation):
        dx = translation[0]
        dy = translation[1]
        pcd[:, 0] += dx
        pcd[:, 1] += dy
        if pcd.shape[1] > 2:
            dz = translation[2]
            pcd[:, 2] -= dz
        return pcd

    def calculate_rotation(self, pcd):
        # extract x0 and y0 arrays from a point cloud array
        x0 = pcd[:, 0]
        y0 = pcd[:, 1]
        # multiply values by a scalar
        x0 = x0 * self.pcd_scalar
        y0 = y0 * self.pcd_scalar
        # set int type
        x0 = x0.astype('int32')
        y0 = y0.astype('int32')
        # max_x = np.max(x0)
        # max_y = np.max(y0)
        # find minimum
        min_x = np.min(x0)
        min_y = np.min(y0)

        x0 = x0 - min_x
        y0 = y0 - min_y
        max_x = np.max(x0)
        max_y = np.max(y0)
        # min_x = np.min(x0)
        # min_y = np.min(y0)
        # print(max_x, max_y)
        # print(min_x, min_y)

        im_arr = np.zeros((max_x + 1, max_y + 1))
        for i in range(len(x0)):
            x = x0[i]
            y = y0[i]
            im_arr[x, y] = 1
            # set_neighbourhood_to_one(im_arr, x, y)

        # apply the Hough transform to find lines in the point cloud
        hough_res, angles, dists = hough_line(im_arr)

        # find the lines with the most votes
        h, angles, dists = hough_line_peaks(hspace=hough_res, angles=angles, dists=dists)
        # print(angles)
        # calculate the rotation of the point cloud based on the lines with the highest votes
        rotation = np.median(angles)

        # print(rotation)

        yaw = rotation
        return yaw

    def calculate_position_with_icp(self, pcd, i):
        # set first point cloud as the reference point cloud
        dx = 0
        dy = 0
        if i == 0:
            self.set_reference_point_cloud(pcd)
        else:
            # ICP
            iter = 0
            while iter < self.max_iterations:
                translation = self.calculate_translation(pcd)
                pcd = self.translate_pcd(pcd, translation)

                dx += translation[0]
                dy += translation[1]
                iter += 1

                # calculate the Euclidean norm
                error = np.linalg.norm(translation)
                print(error)
                if error < self.error:
                    break

        return dx, dy

    def calculate_translation(self, pcd):
        # get an array of closest point to all the points in the point cloud
        arr_of_closest = self.find_array_of_closest_point(pcd)
        pcd_centroid, closest_to_pcd_centroid = self.find_centroids(pcd, arr_of_closest)
        # get translation between the given point cloud and the reference point cloud
        translation = closest_to_pcd_centroid - pcd_centroid
        return translation

    def find_centroids(self, pcd, closest_to_pcd):
        # find centroid of the point cloud
        pcd_centroid = np.mean(pcd, axis=0)
        # find centroid of a point cloud that contains the closest points
        closest_to_pcd_centroid = np.mean(closest_to_pcd, axis=0)
        return pcd_centroid, closest_to_pcd_centroid

    def find_array_of_closest_point(self, pcd):
        closest_to_pcd = np.empty(pcd.shape)
        # find closest point for each point in a point cloud
        for i, point in enumerate(pcd):
            closest_point = self.find_closest_point(point)
            closest_to_pcd[i] = closest_point
        return closest_to_pcd

    def find_closest_point(self, point):
        # calculate distance from a point to each point in reference point cloud
        distance = (self.reference_pcd[:, 0]-point[0])*(self.reference_pcd[:, 0]-point[0]) + (self.reference_pcd[:, 1]-point[1])*(self.reference_pcd[:, 1]-point[1])
        # get index of the closest point
        ind = np.argmin(distance)
        # find coordinates of the closest point
        closest_point = self.reference_pcd[ind]
        return closest_point

    def set_reference_point_cloud(self, pcd):
        self.reference_pcd = pcd

    def save_point_cloud(self, pcd, output_path):
        df = pd.DataFrame(pcd)
        df.to_csv(output_path, index=False)
