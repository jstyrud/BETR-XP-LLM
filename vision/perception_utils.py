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

import cv2
import numpy as np
from typing import Any

def homogeneous_matrix(
    position: np.ndarray,
    orientation: np.ndarray
) -> np.matrix:
    """
    Compute a homogenesous matrix given a translation and rotation.

    Args
    ----
        position: Position of the object.
        orientation: Orientation of the object.
        ros_convention: whether to use ROS msgs or numpy arrays.

    Returns
    -------
        tf_matrix: Homogeneus matrix of the given position and orientation.

    """
    tf_matrix = np.eye(4)
    # translation
    tf_matrix[0, 3] = position[0]
    tf_matrix[1, 3] = position[1]
    tf_matrix[2, 3] = position[2]
    # rotation
    qx = orientation[0]
    qy = orientation[1]
    qz = orientation[2]
    qw = orientation[3]
    tf_matrix[0, 0] = 1-2*(qy**2+qz**2)
    tf_matrix[0, 1] = 2*(qx*qy-qw*qz)
    tf_matrix[0, 2] = 2*(qx*qz+qy*qw)
    tf_matrix[1, 0] = 2*(qx*qy+qw*qz)
    tf_matrix[1, 1] = 1-2*(qx**2+qz**2)
    tf_matrix[1, 2] = 2*(qy*qz-qw*qx)
    tf_matrix[2, 0] = 2*(qx*qz-qw*qy)
    tf_matrix[2, 1] = 2*(qy*qz+qw*qx)
    tf_matrix[2, 2] = 1-2*(qx**2+qy**2)

    return tf_matrix

def get_center(bounding_box: list[float]) -> np.ndarray:
    """Get the center of the bounding box."""
    x_center = int((bounding_box[2] + bounding_box[0])//2)
    y_center = int((bounding_box[3] + bounding_box[1])//2)

    center = np.array([x_center, y_center])
    return center

def pixel_to_point_transform(
    pixel: np.ndarray,
    depth_image: np.ndarray,
) -> np.ndarray:
    """
    Convert 2D pixel into a 3D point.

    Note: this transformation does not take into account the distortion.

    Args
    ----
        pixel: the 2D pixel point to convert.
        depth_image: The depth image to convert to pointcloud, shape (H,W).

    Returns
    -------
        The 3D point in the camera depth frame.

    """
    cx = 327.102478
    cy = 320.964569
    fx = 505.473633
    fy = 505.646698

    px = pixel[0]
    py = pixel[1]

    depth = depth_image[px, py]
    # Depth is in [mm] so we convert in [m]
    # z = depth/1e03
    z = depth
    x = (px - cx)*z/fx
    y = (py - cy)*z/fy

    point_3D = np.array([x/1e03, y/1e03, z/1e03])

    return point_3D

def crop_depth(mask: np.ndarray, depth_img: np.ndarray, cropping: list[int] = None) -> tuple[np.ndarray, list[int]]:
    if mask.shape != depth_img.shape:
        print(f"Mask shape: {mask.shape}")
        print(f"Depth shape: {depth_img.shape}")
        raise ValueError("Mask and Depth image do not have the same shape!")

    depth_masked = cv2.bitwise_and(depth_img, depth_img, mask=mask)

    return depth_masked

def get_points_3D(mask: np.ndarray, depth_img: np.ndarray, camera: Any, transform: np.matrix, cropping: list[int] = None) -> list[np.ndarray]:
    """ Returns 3D points in the mask, downsampling to save compute """
    points_3D = []
    sample_rate = 2
    
    x_y_vals = [(x,y) for x in range(cropping[2], cropping[3], sample_rate) for y in range(cropping[0], cropping[1], sample_rate) if mask[y - cropping[0], x - cropping[2]]]
    for x_y in x_y_vals:
        try:
            point_3D = camera.get_3D_point((x_y[0], x_y[1]), depth_img)
            point_3D_hom = np.append(point_3D, 1)
            transformed_point = transform @ point_3D_hom
            points_3D.append(transformed_point[:3])
        except TypeError:
            continue

    return points_3D

