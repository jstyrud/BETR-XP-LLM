#!/usr/bin/env python3
#
# Copyright (c) 2024, ABB Schweiz AG
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from timeit import default_timer as timer
import cv2
import numpy as np
from matplotlib import pyplot as plt 
from vision.kinect_camera import KinectCamera
import vision.perception_utils as utils
import transforms3d as tf

def get_cube_pose(
    points_3D: list[np.ndarray],
    cube_side: int,
    has_face_threshold: float = 0.3,
    axis_treshold_m: float = 0.005,
    show_debug_images: bool = False
) -> np.ndarray:
    X_OFFSET = 0.002 # general offset in X, maybe can be done better with plane localization etc.
    if show_debug_images:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

    # TOP FACE
    pose_from_top = None
    # step 1: find point with highest Z value
    z_values = [point[2] for point in points_3D]
    index_of_max_z = np.argmax(z_values)
    max_z = points_3D[index_of_max_z][2]
    # step 2a: filter other points within 5mm of that value
    filtered_points = [point for point in points_3D if point[2] >= max_z - axis_treshold_m]
    # step 2b: if less than 30% points remain --> scrap top face
    if len(filtered_points)/len(points_3D) > has_face_threshold:
        # step 3: average remaining points to obtain x,y,z of top face
        points_array = np.vstack(filtered_points)
        average_point = np.mean(points_array, axis=0)
        # step 4: project along -Z by cube_side/2
        pose_from_top = np.array([
            average_point[0] - X_OFFSET, average_point[1], average_point[2] - cube_side/2 + axis_treshold_m / 2])

        if show_debug_images:
            ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], c='r', marker='o')

    # FRONT FACE
    pose_from_front = None
    # step 0_ filter out top and bottom points because they are not reliable in the point cloud
    index_of_min_z = np.argmin(z_values)
    min_z = points_3D[index_of_min_z][2]
    filtered_points = [point for point in points_3D if point[2] <= max_z - axis_treshold_m and point[2] >= min_z + axis_treshold_m]
    # step 1: find point with lowest X value
    x_values = [point[0] for point in filtered_points]
    index_of_min_x = np.argmin(x_values)
    min_x = filtered_points[index_of_min_x][0]
    # step 2a: filter other points within 5mm of that value
    filtered_points = [point for point in filtered_points if point[0] <= min_x + axis_treshold_m]
    # step 2b: if less than 30% points remain --> scrap front face
    if len(filtered_points)/len(points_3D) > has_face_threshold:
        # step 3: average remaining points to obtain x,y,z of front face
        points_array = np.vstack(filtered_points)
        average_point = np.mean(points_array, axis=0)
        # step 4: project along +X by cube_side/2
        pose_from_front = np.array([
            average_point[0] + cube_side/2 - axis_treshold_m / 2 - X_OFFSET, average_point[1], average_point[2]])

        if show_debug_images:
            ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], c='b', marker='o')
    if show_debug_images:
        plt.show()

    # final step: if both faces available then do average to obtain center
    if pose_from_top is None and pose_from_front is None:
        return None
    elif pose_from_top is None:
        return pose_from_front + np.array([0.0, 0.0, 0.005]) # Generally it's off when top view is not visible because of point cloud smearing
    elif pose_from_front is None:
        return pose_from_top
    else:
        avg_pose = pose_from_front
        avg_pose[1] = (pose_from_front[1] + pose_from_top[1]) / 2
        avg_pose[2] = (pose_from_front[2] + pose_from_top[2]) / 2
        return avg_pose


def cube_heuristic(
    mask: np.ndarray,
    depth_img: np.ndarray,
    rgb_img: np.ndarray,
    camera: KinectCamera,
    transformation: np.matrix, 
    cube_side_m: int,
    cropping: list[int] = None,
    has_face_threshold: float = 0.2,
    axis_treshold_m: float = 0.005
) -> np.ndarray:
    start = timer()
    points_3D = utils.get_points_3D(mask, depth_img, camera, transformation, cropping)
    print(f"Got 3D points in {timer() - start:.2f}s")
    start = timer()
    pose = get_cube_pose(points_3D, cube_side_m, has_face_threshold, axis_treshold_m)
    print(f"Got cube pose in {timer() - start:.2f}s")
    return pose


def get_cup_flipped(
    points_3D: list[np.ndarray],
    cluster_size_threshold: int = 1000,
    cluster_height_threshold_m: float = 0.005,
    show_debug_images: bool = False 
) -> bool:
    is_flipped = False
    # TOP FACE
    # step 1: find point with highest Z value
    z_values = [point[2] for point in points_3D]
    index_of_max_z = np.argmax(z_values)
    max_z = points_3D[index_of_max_z][2]
    # step 2a: filter other points within 5mm of that value
    filtered_points = [point for point in points_3D if point[2] >= max_z - cluster_height_threshold_m]
    # step 2b: if cluster large enough then the cup is upside down
    if len(filtered_points) > cluster_size_threshold:
        is_flipped = True

    return is_flipped


def cup_heuristic(
    mask: np.ndarray,
    depth_img: np.ndarray,
    camera: KinectCamera,
    transformation: np.matrix,
    cropping: list[int] = None,
    show_debug_images: bool = False
) -> tuple[bool, np.ndarray]:
    start = timer()
    points_3D = utils.get_points_3D(mask, depth_img, camera, transformation, cropping)
    print(f"Got 3D points in {timer() - start:.2f}s")
    is_flipped = get_cup_flipped(points_3D)
    
    start = timer()
    points_array = np.vstack(points_3D)
    pose = np.mean(points_array, axis=0)
    if is_flipped:
        pose += np.array([-0.01, 0.0, 0.0]) # For some reason the points for the cup are offset in the x-direction, and more so if not flipped
    else:
        pose += np.array([-0.015, -0.005, 0.0])
    print(f"Got cup pose in {timer() - start:.2f}s")

    if show_debug_images:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], c='r', marker='o')
        plt.show()
    
    return pose, is_flipped

def cap_heuristic(
    mask: np.ndarray,
    depth_img: np.ndarray,
    camera: KinectCamera,
    transformation: np.matrix,
    cropping: list[int] = None,
    show_debug_images: bool = False
) -> tuple[bool, np.ndarray]:
    start = timer()
    points_3D = utils.get_points_3D(mask, depth_img, camera, transformation, cropping)
    print(f"Got 3D points in {timer() - start:.2f}s")

    # find point with highest Z value
    z_values = [point[2] for point in points_3D]
    index_of_max_z = np.argmax(z_values)
    max_z = points_3D[index_of_max_z][2]

    # find point with lowest X value
    x_values = [point[0] for point in points_3D]
    index_of_min_x = np.argmin(x_values)
    min_x = points_3D[index_of_min_x][0]
    
    start = timer()
    points_array = np.vstack(points_3D)
    pose = np.mean(points_array, axis=0)
     # Caps has some repetivie offsets as well
    pose[0] = min_x - 0.01
    pose[1] += 0.01
    pose[2] = max_z + 0.01
    print(f"Got cap pose in {timer() - start:.2f}s")

    if show_debug_images:
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], c='r', marker='o')
        plt.show()
    
    return pose


def centrifuge_heuristic(mask: np.ndarray) -> bool:
    # converting mask into grayscale image 
    plt.imsave("mask.png", mask)
    img = cv2.imread('mask.png') 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    # setting threshold of gray image 
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
    # using a findContours() function 
    contours, _ = cv2.findContours( 
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    try:
        max_contour = max(contours[1:], key=lambda c: c.size)
    except ValueError:
        return True
    print("Contour size: ", max_contour.size)
    print("Mask size: ", np.sum(mask))
    is_open = False
    # Throw heuristic
    if max_contour.size > 2400 or np.sum(mask) > 470000:
        is_open = True
    else:
        is_open = False

    return is_open


def table_benchmark(
    depth_img: np.ndarray,
    camera: KinectCamera,
    transformation: np.matrix,
    cropping: list[int] = None
) -> None:
    lower = 790
    upper = 810

    if cropping is not None:
        depth_cropped = depth_img[cropping[0]:cropping[1], cropping[2]:cropping[3]]
    depth_very_cropped = depth_cropped[lower:upper, lower:upper]
    
    
    points_3D = []
    points_3D_tfd = []
    for x in range(lower,upper):
        for y in range(lower,upper):
            pixel = (x + 1500, y + 1000)
            try:
                point_3D = camera.get_3D_point(pixel, depth_img)
                point_3D_hom = np.append(point_3D, 1)
                transformed_point = transformation @ point_3D_hom
                points_3D_tfd.append(transformed_point[:3])
                points_3D.append(point_3D)
            except TypeError:
                continue

    # print depth points in 3D space in camera frame
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    for point in points_3D:
            ax.scatter(point[0], point[1], point[2], c='r', marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    plt.show()
    
    # print depth points in 3D space in robot base frame
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    for point in points_3D_tfd:
            ax.scatter(point[0], point[1], point[2], c='r', marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    plt.show()
