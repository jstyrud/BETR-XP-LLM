""" Vision system for object detection, segmentation and pose estimation from RGB-D images """
# Copyright (c) 2024, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
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
import os
import os.path as osp
from timeit import default_timer as timer
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import torch
import torchvision
from mmengine.config import Config
from mmengine.dataset import Compose
from mmdet.apis import init_detector
from mmdet.utils import get_test_pipeline_cfg

import supervision as sv
import PIL.Image
from nanosam.utils.predictor import Predictor
from vision.kinect_camera import KinectCamera
import vision.perception_utils as utils
import transforms3d
import math
import vision.heuristics as heur

#Settings
object_color_dict = {
    "green": ((60, 20, 0), (90, 255, 150)),
    "blue": ((100, 50, 0), (150, 255, 200)),
    "purple": ((160, 70, 0), (180, 200, 110)),
    "yellow": ((12, 130, 120), (40, 255, 250)),
    "orange": ((0, 130, 120), (11, 255, 255)),
    "red": ((165, 150, 100), (255, 250, 255)),
}

class VisionSystem():
    """ Main vision system object for preloading models etc. """
    def __init__(self,
                 config_path="YOLO-World/configs/pretrain/yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py",# pylint: disable=line-too-long
                 checkpoint_path="YOLO-World/weights/yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth",
                 device="cuda:0",
                 output_dir="demo_outputs"):
        """ Initializes and saves parameters """
        self.config_path=config_path
        self.checkpoint_path=checkpoint_path
        self.device=device

        self.output_dir = output_dir
        if not osp.exists(output_dir):
            os.mkdir(output_dir)

        self.box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.mask_annotator = sv.MaskAnnotator()

        # Just declare some model objects
        self.cfg = None
        self.model = None
        self.test_pipeline = None
        self.sam_predictor = None

    def load_models(self):
        """ Loads the models that will be reused later """

        # Load YOLO-World models
        self.cfg = Config.fromfile(self.config_path)
        self.cfg.load_from = self.checkpoint_path
        self.model = init_detector(self.cfg, checkpoint=self.checkpoint_path, device=self.device)

        # init YOLO-World test pipeline
        test_pipeline_cfg = get_test_pipeline_cfg(cfg=self.cfg)
        test_pipeline_cfg[0].type = 'mmdet.LoadImageFromNDArray'
        self.test_pipeline = Compose(test_pipeline_cfg)

        # Instantiate TensorRT predictor for nanosam
        self.sam_predictor = Predictor("nanosam/data/resnet18_image_encoder.engine",
                                       "nanosam/data/mobile_sam_mask_decoder.engine")

    def yolo_world_object_detection(self, image, texts, score_thr, max_dets):
        """ Runs object detection based on YOLO-world """
        # Do the prediction
        start = timer()
        data_info = dict(img_id=0, img=image, texts=texts)
        data_info = self.test_pipeline(data_info)
        data_batch = dict(inputs=data_info['inputs'].unsqueeze(0),
                    data_samples=[data_info['data_samples']])

        with torch.no_grad():
            output = self.model.test_step(data_batch)[0]
            pred_instances = output.pred_instances
            pred_instances = pred_instances[pred_instances.scores.float() >
                                            score_thr]
        print(f"Model step in {timer() - start:.2f}s")
        
        # Only one object per label
        indices = []
        used_labels = []
        for i in range(len(pred_instances.labels)):
            if pred_instances.labels[i] not in used_labels:
                indices.append(i)
                used_labels.append(pred_instances.labels[i])
        pred_instances = pred_instances[indices]

        if len(pred_instances.scores) > max_dets:
            indices = pred_instances.scores.float().topk(max_dets)[1]
            pred_instances = pred_instances[indices]

        return pred_instances.cpu().numpy()

    @staticmethod
    def extract_colors(texts):
        """ Extracts colors from texts"""
        found_colors = []
        for color in object_color_dict:
            for text in texts:
                if text[0][0:len(color) + 1] == color + " ":
                    found_colors.append(color)
                    text[0] = text[0][len(color) + 1:]
                    break

        # Remove duplicates after colors were removed
        unique_texts = []
        for text in texts:
            if text not in unique_texts:
                unique_texts.append(text)
        return found_colors, unique_texts

    def detect_objects(self,
                       image,
                       texts,
                       score_thr=0.3,
                       nms_thr=0.7,
                       max_dets=10,
                       do_color_masking=False,
                       show_debug_images=False):
        """ Run YOLO-world open vocabulary object detection """

        # Format the texts input to fit YOLO-world requirements
        if isinstance(texts, str):
            texts = [[t.strip()] for t in texts.split(',')] + [[' ']]
        elif isinstance(texts, list):
            for i, text in enumerate(texts):
                texts[i] = [text.replace('"', '')]
            texts.append([' '])
        if do_color_masking:
            found_colors, texts = self.extract_colors(texts)

        # Reparameterize texts
        self.model.reparameterize(texts)

        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rgb = image
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        if show_debug_images:
            plt.subplot(2, 4, 1)
            plt.imshow(image_rgb)
        index = 1
        xyxy = []
        confidence = []
        class_id = []
        labels = []
        object_labels = []
        if do_color_masking and found_colors:
            for object_color in found_colors: # pylint: disable=consider-using-dict-items,
                # Mask out the color
                mask = cv2.inRange(image_hsv, object_color_dict[object_color][0], object_color_dict[object_color][1])
                result = cv2.bitwise_and(image_rgb, image_rgb, mask=mask)
                image_masked = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)

                pred_instances = self.yolo_world_object_detection(image_masked, texts, score_thr, max_dets=1)

                if index > 1:
                    xyxy = np.concatenate((xyxy, pred_instances['bboxes']))
                    confidence = np.concatenate((confidence, pred_instances['scores']))
                    class_id = np.concatenate((class_id, pred_instances['labels'])) + index * 100
                else:
                    xyxy = pred_instances['bboxes']
                    confidence = pred_instances['scores']
                    class_id = pred_instances['labels'] + index * 100

                labels += [f"{object_color + ' ' + texts[class_id][0]} {confidence:0.2f}"
                            for class_id, confidence in
                            zip(pred_instances['labels'], pred_instances['scores'])]
                object_labels += [f"{object_color + ' ' + texts[class_id][0]}"
                                  for class_id in pred_instances['labels']]

                index += 1
                if show_debug_images:
                    plt.subplot(2, 4, index)
                    plt.imshow(result)
        else:
            pred_instances = self.yolo_world_object_detection(image_rgb, texts, score_thr, max_dets)
            xyxy = pred_instances['bboxes']
            confidence = pred_instances['scores']
            class_id = pred_instances['labels']
            labels += [f"{texts[class_id][0]} {confidence:0.2f}"
            for class_id, confidence in
            zip(pred_instances['labels'], pred_instances['scores'])]
            object_labels += [f"{texts[class_id][0]}"
            for class_id in pred_instances['labels']]

        detections = sv.Detections(xyxy=xyxy, confidence=confidence, class_id=class_id)
        labels = np.array(labels)
        object_labels = np.array(object_labels)

        # NMS post process
        print(f"Before NMS: {len(detections.xyxy)} boxes")
        nms_idx = torchvision.ops.nms(
            torch.from_numpy(detections.xyxy),
            torch.from_numpy(detections.confidence),
            nms_thr
        ).numpy().tolist()

        detections.xyxy = detections.xyxy[nms_idx]
        detections.confidence = detections.confidence[nms_idx]
        detections.class_id = detections.class_id[nms_idx]
        labels = labels[nms_idx]
        object_labels = object_labels[nms_idx]

        if show_debug_images:
            plt.show(block=False)

            pixel_colors = image_rgb.reshape((np.shape(image_rgb)[0]*np.shape(image_rgb)[1], 3))
            norm = colors.Normalize(vmin=-1.,vmax=1.)
            norm.autoscale(pixel_colors)
            pixel_colors = norm(pixel_colors).tolist()

            h, s, v = cv2.split(image_hsv)
            fig = plt.figure()
            axis = fig.add_subplot(1, 1, 1, projection="3d")

            axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
            axis.set_xlabel("Hue")
            axis.set_ylabel("Saturation")
            axis.set_zlabel("Value")
            plt.show()
        return detections, labels, object_labels

    @staticmethod
    def bbox2points(bbox):
        """ Converts bbox to proper format for sam predictor """
        points = np.array([
            [bbox[0], bbox[1]],
            [bbox[2], bbox[3]]
        ])

        point_labels = np.array([2, 3])

        return points, point_labels

    def segment(self, image, xyxy):
        """ Run sam segmentation and return segmentation masks"""
        self.sam_predictor.set_image(image)
        result_masks = []
        for box in xyxy:
            points, point_labels = self.bbox2points(box)
            mask, _, _ = self.sam_predictor.predict(points, point_labels)

            result_masks.append(mask[0, 0] > 0)
        return [t.cpu().numpy() for t in result_masks]
    
    def fill_holes_and_erode_mask(self, mask, show_debug_images=False):
        """ Fills any holes and erodes bounday of mask """
        # Fill holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        holes_filled = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if show_debug_images:
            plt.figure()
            plt.imshow(holes_filled)
            plt.show(block=False)

        # Erode
        kernel = np.ones((5, 5), np.uint8) 
        eroded = cv2.erode(holes_filled, kernel, iterations=1) 
        if show_debug_images:
            plt.figure()
            plt.imshow(eroded)
            plt.show()
        return eroded

    def run_pose_estimation_pipeline(
        self,
        image,
        image_path="",
        texts="",
        max_dets=10,
        score_thr=0.3,
        nms_thr = 0.7,
        do_color_masking=False,
        show_debug_images=False
    ):
        """ Run the pose estimation pipeline step by step"""
        start = timer()
        if image is None:
            image = cv2.imread(image_path)
        print(f"Image read in {timer() - start:.2f}s")
        start = timer()
        detections, labels, object_labels = self.detect_objects(image=image, texts=texts,
                                                                score_thr=score_thr,
                                                                nms_thr=nms_thr, max_dets=max_dets,
                                                                do_color_masking=do_color_masking,
                                                                show_debug_images=show_debug_images)
        print(f"Objects detected in {timer() - start:.2f}s")
        start = timer()
        if image is None:
            pil_image = PIL.Image.open(image_path).convert('RGB')
        else:
            pil_image = PIL.Image.fromarray(image)

        # convert detections to masks
        detections.mask = self.segment(image=pil_image,
                                       xyxy=detections.xyxy)
        
        detections.int_mask = []
        for mask in detections.mask:
            detections.int_mask.append(self.fill_holes_and_erode_mask(mask.astype(np.uint8), show_debug_images))

        print(f"Segmentation done in in {timer() - start:.2f}s")
        annotated_image = self.mask_annotator.annotate(image.copy(), detections=detections)
        annotated_image = self.box_annotator.annotate(annotated_image, detections=detections)
        annotated_image = self.label_annotator.annotate(annotated_image, detections, labels)
        cv2.imwrite(osp.join(self.output_dir, osp.basename(image_path)), annotated_image)

        return detections.int_mask, object_labels

if __name__ == '__main__':
    start = timer()
    vision_system = VisionSystem()
    vision_system.load_models()
    print(f"Models loaded in {timer() - start:.1f}s")

    # take pictures
    camera = KinectCamera()
    cropping = [1500, 3072, 1000, 3000]
    
    rgb_img, depth_img, _  = camera.get_image(cropping)

    # compute transformation of camera in robot frame
    pos = np.array([0.117, -0.038, 0.647])
    quat = np.array([-0.645, 0.653, -0.273, 0.288])
    T_camera_in_robot = utils.homogeneous_matrix(pos, quat)
     
    target_object = "cup"
    # run pose estimation pipeline
    mask, label = vision_system.run_pose_estimation_pipeline(
        image=rgb_img,
        image_path="cube_images/matteo3.png",
        texts=target_object,
        max_dets=1,
        score_thr=0.2,
        do_color_masking=True,
        show_debug_images=False
    )

    print(f"Found: {label}")

    if "cup" in target_object:
        cup_pose, is_flipped = heur.cup_heuristic(mask[0], depth_img, rgb_img, camera, T_camera_in_robot, cropping)
        status = "flipped" if is_flipped else "not flipped"
        print(f"Cup {status} at pose: {cup_pose}")
    elif "cube" in target_object:
        cube_pose = heur.cube_heuristic(mask[0], depth_img, rgb_img, camera, T_camera_in_robot, 0.025, cropping)
        print(f"Cube pose: {cube_pose}")
    elif "centrifuge" in target_object:
        is_open = heur.centrifuge_heuristic(mask[0])
        status = "open" if is_open else "close"
        print(f"Centrifuge {status}!")
