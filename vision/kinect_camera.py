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

import vision.k4a as k4a
import imageio
import matplotlib.pyplot as plt
import typing
import numpy as np
import time
import cv2
from vision.camera_base import CameraBase


class KinectCamera(CameraBase):
    PIXEL_BOUNDS = [1300, 2500, 1000, 2700]

    CONFIG = k4a.DeviceConfiguration(
        color_format=k4a.EImageFormat.COLOR_BGRA32,
        color_resolution=k4a.EColorResolution.RES_3072P,
        depth_mode=k4a.EDepthMode.NFOV_UNBINNED,
        camera_fps=k4a.EFramesPerSecond.FPS_5,
        synchronized_images_only=True,
        depth_delay_off_color_usec=0,
        wired_sync_mode=k4a.EWiredSyncMode.STANDALONE,
        subordinate_delay_off_master_usec=0,
        disable_streaming_indicator=False
    )

    def __init__(self,
                position=(0.117, -0.038, 0.647),
                orientation=(-2.326, 0.024, -1.574)):
        """
        ros2 run tf2_ros tf2_echo ORWorkStation_yumi_base_link ORWorkStation_yumi_rgb_camera_link
        - Translation: [0.117, -0.038, 0.647]
        - Rotation: in Quaternion [-0.645, 0.653, -0.273, 0.288]
        - Rotation: in RPY (radian) [-2.326, 0.024, -1.574]
        - Rotation: in RPY (degree) [-133.274, 1.356, -90.158]
        """
        self._saturation_coef = 1.0
        self._device = None
        while not self._device:
            self._device = k4a.Device.open()
            time.sleep(1)
        while self._device.start_cameras(self.CONFIG) == k4a.EStatus.FAILED:
            time.sleep(1)
            print("Failed to start cameras, retrying...")
        print("Kinect device started.")
        self._device.set_color_control(
            k4a.EColorControlCommand.EXPOSURE_TIME_ABSOLUTE,
            k4a.EColorControlMode.AUTO,
            3000
        )
        self._device.set_color_control(
            k4a.EColorControlCommand.WHITEBALANCE,
            k4a.EColorControlMode.AUTO,
            4500
        )
        self._device.set_color_control(
            k4a.EColorControlCommand.BACKLIGHT_COMPENSATION,
            k4a.EColorControlMode.AUTO,
            1
        )
        self._device.set_color_control(
            k4a.EColorControlCommand.POWERLINE_FREQUENCY,
            k4a.EColorControlMode.MANUAL,
            1
        )
        self._calibration = self._device.get_calibration(
            self.CONFIG.depth_mode,
            self.CONFIG.color_resolution
        )
        self._transformation = k4a.Transformation(self._calibration)
        self._position = position
        self._orientation = orientation
        super().__init__(0.002)

        self.bounds = np.float32([[0.15, 0.75], [-0.3, 0.3], [-0.15, 0.1]])
        # Perform auto white balance
        capture = None
        while not capture:
            capture = self._device.get_capture(-1)
        for i in range(5):
            self._device.get_capture(-1)


    def close(self):
        self._device.stop_cameras()

    def plot_images(self, color_img:k4a.Image, depth_img:k4a.Image, trans_img:k4a.Image, cmap="jet"):
        fig = plt.figure()
        ax = []
        ax.append(fig.add_subplot(1,3,1,label="RGB"))
        ax.append(fig.add_subplot(1,3,2,label="Depth"))
        ax.append(fig.add_subplot(1,3,3,label="Transformed"))

        im = []
        im.append(ax[0].imshow(color_img.data))
        im.append(ax[1].imshow(depth_img.data, cmap=cmap))
        im.append(ax[2].imshow(trans_img.data, cmap=cmap))

        ax[0].title.set_text("RGB")
        ax[1].title.set_text("Depth")
        ax[2].title.set_text("Transformed")

        plt.show()

    def get_image(self, cropping: list[int] = None):
        capture = None
        while not capture:
            capture = self._device.get_capture(-1)
        bgr = capture.color.data[:, :, :3].copy()
        rgb_img = bgr[..., ::-1]

        # transform depth data in color space
        depth_raw = self._transformation.depth_image_to_color_camera(capture.depth)
        depth_img = depth_raw.data

        # crop rgb image
        h, w, c = rgb_img.shape
        if cropping is not None:
            if cropping[1] > h or cropping[3] > w:
                raise ValueError(f"Cropping values higher than image shape: {h, w}")
            rgb_img_cropped = rgb_img[cropping[0]:cropping[1], cropping[2]:cropping[3], :]
        rgb_img_cropped = self.increase_saturation(rgb_img_cropped, self._saturation_coef)
        
        if cropping is not None:
            if cropping[1] > h or cropping[3] > w:
                raise ValueError(f"Cropping values higher than image shape: {h, w}")
            cropped_depth_img = depth_img[cropping[0]:cropping[1], cropping[2]:cropping[3]]
        else:
            cropped_depth_img = depth_img
        h, w = cropped_depth_img.shape
        cropped_depth_img = cropped_depth_img.astype(float)

        depth_img = depth_img.astype(float)
        
        # save images
        imageio.imsave("./cube_images/rgb.png", rgb_img_cropped)

        return rgb_img_cropped, depth_img, cropped_depth_img

    def get_depth_image(self):
        return self.get_image()[1]

    def get_rgb_image(self):
        return self.get_image()[0]
    
    def get_3D_point(self, pixel: tuple, depth_image: np.ndarray):
        # Apply smoothening by looking at the neighboring depth values
        depth_mm = depth_image[pixel[1],pixel[0]]
        (x, y, z) = self._transformation.pixel_2d_to_point_3d(
            pixel, depth_mm, k4a.ECalibrationType.COLOR, k4a.ECalibrationType.COLOR)
        return np.array([x/1e03, y/1e03, z/1e03])
    
    @staticmethod
    def increase_saturation(img, coefficient):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        hsv[...,1] = hsv[...,1] * coefficient
        img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        return np.clip(img, 0, 255)

    def get_observation(self, picks: list, places: list) -> typing.Dict:
        observation = {}

        capture = self._device.get_capture(-1)
        bgr = capture.color.data[:, :, :3].copy()
        rgb = self.increase_saturation(bgr[..., ::-1], self._saturation_coef)
        depth = self._transformation.depth_image_to_color_camera(capture.depth)
        points = self._transformation.depth_image_to_point_cloud(depth, k4a.ECalibrationType.COLOR)
        valid_points = points.data.copy() / 1000.0
        valid_depth = depth.data.copy()

        position = np.float32(self._position).reshape(3, 1)
        rotation = self._rpy_to_matrix(self._orientation)
        transform = np.eye(4)
        transform[:3, :] = np.hstack((rotation, position))
        transformed_points = self._transform_pointcloud(valid_points, transform)
        heightmap, colormap, xyzmap = self._get_heightmap(transformed_points, rgb,
                                                          self.bounds, self.pixel_size)

        observation["points"] = transformed_points
        observation["color"] = rgb
        observation["depth"] = valid_depth
        observation["heightmap"] = heightmap
        observation["image"] = colormap
        observation["xyzmap"] = xyzmap
        observation["pick"] = picks
        observation["place"] = places
        return observation

    @staticmethod
    def _rpy_to_matrix(coords):
        coords = np.asanyarray(coords, dtype=np.float64)
        c3, c2, c1 = np.cos(coords)
        s3, s2, s1 = np.sin(coords)

        return np.array([
            [c1 * c2, (c1 * s2 * s3) - (c3 * s1), (s1 * s3) + (c1 * c3 * s2)],
            [c2 * s1, (c1 * c3) + (s1 * s2 * s3), (c3 * s1 * s2) - (c1 * s3)],
            [-s2, c2 * s3, c2 * c3]
        ], dtype=np.float64)
    
    @staticmethod
    def _depth_avg_filter(pixel: tuple, depth_image: np.ndarray, neighbors: int=2):
        px = int(pixel[0])
        py = int(pixel[1])
        depth_queue = []
        # Take depth average in the surrounding area of 4x4 pixels
        for x_ in range(neighbors):
            for y_ in range(neighbors):
                try:
                    depth_queue.append(depth_image[py + (y_ + 1)][px + (x_ + 1)])
                except IndexError:
                    continue
                try:
                    depth_queue.append(depth_image[py + (y_ + 1)][px - (x_ + 1)])
                except IndexError:
                    continue
                try:
                    depth_queue.append(depth_image[py - (y_ + 1)][px + (x_ + 1)])
                except IndexError:
                    continue
                try:
                    depth_queue.append(depth_image[py - (y_ + 1)][px - (x_ + 1)])
                except IndexError:
                    continue
                # Remove 0 values
                if 0 in depth_queue:
                    depth_queue.remove(0)
        # The depth is in [mm]
        return sum(depth_queue)/len(depth_queue)


