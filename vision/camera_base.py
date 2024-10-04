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

import typing
from datetime import datetime
from abc import ABCMeta
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import cv2


class CameraBase(metaclass=ABCMeta):
    def __init__(self, pixel_size=0.00267857):
        self.bounds = np.float32([[-0.3, 0.3], [-0.8, -0.2], [0, 0.15]])  # X Y Z
        self.pixel_size = pixel_size
        self.obs_width = int((self.bounds[0][1] - self.bounds[0][0])/self.pixel_size)
        self.obs_height = int((self.bounds[1][1] - self.bounds[1][0]) / self.pixel_size)

    def get_image(self) -> np.array:
        pass

    def get_vild_image(self) -> np.array:
        return np.flipud(self.get_image().transpose(1, 0, 2))

    def get_observation(self, picks: list, places: list) -> typing.Dict:
        return {}

    @staticmethod
    def plot(img: np.array, filename=str(), block=True):
        now = datetime.now()
        date_time = now.strftime("%m-%d-%Y,%H:%M:%S")
        plt.imshow(img)
        if len(filename):
            im_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(f"{filename}-{date_time}.png", im_bgr)
            # plt.savefig()
        plt.show(block=block)

    @staticmethod
    def plot_obs(obs: typing.Dict, filename=str(), block=True):
        fig, plts = plt.subplots(2, 3)
        fig.set_size_inches(8, 8)
        plts[0, 0].set_title("Color")
        plts[0, 0].imshow(obs["color"])
        plts[0, 1].set_title("Depth")
        plts[0, 1].imshow(obs["depth"])
        plts[0, 2].set_title("Heightmap")
        plts[0, 2].imshow(obs["heightmap"])
        plts[1, 0].set_title("Colormap")
        plts[1, 0].imshow(obs["image"])
        plts[1, 1].set_title("xyzmap")
        CameraBase.plot_normalize(plts[1, 1], obs["xyzmap"])
        plts[1, 2].set_title("points")
        CameraBase.plot_normalize(plts[1, 2], obs["points"])

        if len(filename):
            plt.savefig(filename)
        plt.show(block=block)

    @staticmethod
    def plot_normalize(plot, all_data):
        # create your own colormap
        data = all_data[:, :, 2]
        num = 100  # how many colors do you need?
        pmax, pmin = np.max(data), np.min(data)  # get data range
        bounds = np.linspace(pmin, pmax, num)  # set data range for colormap
        norm = colors.BoundaryNorm(bounds, ncolors=num)  # an object for cmap normalization
        carray = np.array([np.linspace(0, 1, num), np.linspace(0.0, 0.25, num), [0.5] * num,
                           [1] * num]).T  # list of RGBA values
        cmap = colors.ListedColormap(carray, N=num,
                                     name='mycolormap')  # create your own color map from carray
        plot.imshow(data, interpolation="nearest", origin="lower", cmap=cmap, norm=norm)

    @staticmethod
    def _get_pointcloud(depth, intrinsics):
        """Get 3D pointcloud from perspective depth image.
        Args:
          depth: HxW float array of perspective depth in meters.
          intrinsics: 3x3 float array of camera intrinsics matrix.
        Returns:
          points: HxWx3 float array of 3D points in camera coordinates.
        """
        height, width = depth.shape
        xlin = np.linspace(0, width - 1, width)
        ylin = np.linspace(0, height - 1, height)
        px, py = np.meshgrid(xlin, ylin)
        px = (px - intrinsics[0, 2]) * (depth / intrinsics[0, 0])
        py = (py - intrinsics[1, 2]) * (depth / intrinsics[1, 1])
        points = np.float32([px, py, depth]).transpose(1, 2, 0)
        return points

    @staticmethod
    def _transform_pointcloud(points, transform):
        """Apply rigid transformation to 3D pointcloud.
        Args:
          points: HxWx3 float array of 3D points in camera coordinates.
          transform: 4x4 float array representing a rigid transformation matrix.
        Returns:
          points: HxWx3 float array of transformed 3D points.
        """
        padding = ((0, 0), (0, 0), (0, 1))
        homogen_points = np.pad(points.copy(), padding, "constant", constant_values=1)
        for i in range(3):
            points[Ellipsis, i] = np.sum(transform[i, :] * homogen_points, axis=-1)
        return points

    @staticmethod
    def _get_heightmap(points, colors, bounds, pixel_size):
        """Get top-down (z-axis) orthographic heightmap image from 3D pointcloud.
        Args:
          points: HxWx3 float array of 3D points in world coordinates.
          colors: HxWx3 uint8 array of values in range 0-255 aligned with points.
          bounds: 3x2 float array of values (rows: X,Y,Z; columns: min,max) defining
            region in 3D space to generate heightmap in world coordinates.
          pixel_size: float defining size of each pixel in meters.
        Returns:
          heightmap: HxW float array of height (from lower z-bound) in meters.
          colormap: HxWx3 uint8 array of backprojected color aligned with heightmap.
          xyzmap: HxWx3 float array of XYZ points in world coordinates.
        """
        # height, width, channels = colors.shape
        width = int(np.round((bounds[0, 1] - bounds[0, 0]) / pixel_size))
        height = int(np.round((bounds[1, 1] - bounds[1, 0]) / pixel_size))
        heightmap = np.zeros((height, width), dtype=np.float32)
        colormap = np.zeros((height, width, colors.shape[-1]), dtype=np.uint8)
        xyzmap = np.zeros((height, width, 3), dtype=np.float32)

        # Filter out 3D points that are outside the predefined bounds.
        ix = (points[Ellipsis, 0] >= bounds[0, 0]) & (points[Ellipsis, 0] < bounds[0, 1])
        iy = (points[Ellipsis, 1] >= bounds[1, 0]) & (points[Ellipsis, 1] < bounds[1, 1])
        iz = (points[Ellipsis, 2] >= bounds[2, 0]) & (points[Ellipsis, 2] < bounds[2, 1])
        valid = ix & iy & iz
        points = points[valid]
        colors = colors[valid]

        # Sort 3D points by z-value, which works with array assignment to simulate
        # z-buffering for rendering the heightmap image.
        iz = np.argsort(points[:, -1])
        points, colors = points[iz], colors[iz]
        px = np.int32(np.floor((points[:, 0] - bounds[0, 0]) / pixel_size))
        py = np.int32(np.floor((points[:, 1] - bounds[1, 0]) / pixel_size))
        px = np.clip(px, 0, width - 1)
        py = np.clip(py, 0, height - 1)
        heightmap[py, px] = points[:, 2] - bounds[2, 0]
        for c in range(colors.shape[-1]):
            colormap[py, px, c] = colors[:, c]
            xyzmap[py, px, c] = points[:, c]
        colormap = colormap[::-1, :, :]  # Flip up-down.
        xv, yv = np.meshgrid(np.linspace(bounds[0, 0], bounds[0, 1], width),
                             np.linspace(bounds[1, 0], bounds[1, 1], height))
        xyzmap[:, :, 0] = xv
        xyzmap[:, :, 1] = yv
        xyzmap = xyzmap[::-1, :, :]  # Flip up-down.
        heightmap = heightmap[::-1, :]  # Flip up-down.
        return heightmap, colormap, xyzmap
