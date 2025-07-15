#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime
from typing import List, Tuple, Optional

import cv2
import numpy as np
import rospy
import supervision as sv
from ultralytics import YOLO

from robot_perception.msg import Identify, Point
from robot_scheduler.msg import SchedulerCommand  

class VisionDetector:
    def __init__(self, model_path: str, conf_thres: float = 0.5):
        self.model = YOLO(model_path)
        self.model.overrides["verbose"] = False
        self.conf_thres = conf_thres

        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator(text_scale=0.5,
                                                 text_thickness=1)

        self.cx = 640 / 2
        self.cy = 480 / 2
        self.tolerancia = 5

    def infer(self, frame: np.ndarray):
        result = self.model(frame, imgsz=640, conf=self.conf_thres)[0]
        detections = sv.Detections.from_ultralytics(result)

        angles = [self._angle_from_bbox(frame, xyxy) for xyxy in detections.xyxy]

        labels = [
            f"{self.model.names[c]} {p:.2f}, ang: "
            f"{a:.2f}" if a is not None else
            f"{self.model.names[c]} {p:.2f}, ang: N/A"
            for c, p, a in zip(detections.class_id,
                               detections.confidence,
                               angles)
        ]

        annotated = self.box_annotator.annotate(frame.copy(), detections)
        annotated = self.label_annotator.annotate(annotated, detections, labels)

        cv2.rectangle(
            annotated,
            (int(self.cx - self.tolerancia), int(self.cy - self.tolerancia)),
            (int(self.cx + self.tolerancia), int(self.cy + self.tolerancia)),
            (255, 255, 0), 1)

        centroids = [((x1+x2)/2, (y1+y2)/2) for x1, y1, x2, y2 in detections.xyxy]
        for (cx, cy) in centroids:
            color = (0, 255, 0) if abs(cx - self.cx) <= self.tolerancia \
                                 and abs(cy - self.cy) <= self.tolerancia else (0, 0, 255)
            cv2.circle(annotated, (int(cx), int(cy)), 5, color, -1)

        return annotated, \
               [self.model.names[c] for c in detections.class_id], \
               centroids, \
               angles

    @staticmethod
    def _angle_from_bbox(frame: np.ndarray, xyxy) -> Optional[float]:
        x1, y1, x2, y2 = map(int, xyxy)

        h, w = frame.shape[:2]
        x1, x2 = np.clip([x1, x2], 0, w-1)
        y1, y2 = np.clip([y1, y2], 0, h-1)
        if x2 <= x1 or y2 <= y1:
            return None

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        ys, xs = np.nonzero(edges)
        coords = np.column_stack((xs, ys))
        if len(coords) < 5:
            return None

        _, vecs = cv2.PCACompute(coords.astype(np.float32), mean=None)
        angle = np.degrees(np.arctan2(vecs[0, 1], vecs[0, 0]))
        angle = abs(angle) % 180
        return angle

class VisionIdentificationNode:
    def __init__(self):
        rospy.init_node("vision_identification")

        root_dir   = rospy.get_param("~root_dir", os.path.dirname(__file__))
        self.model = rospy.get_param("~model", "models/main_model.pt")
        self.cam_device = rospy.get_param("~camera_device", "0")
        self.conf   = float(rospy.get_param("~conf_thresh", 0.5))
        self.gui    = bool(rospy.get_param("~display", True))
        self.snap_dir = os.path.join(root_dir, "imagens")
        os.makedirs(self.snap_dir, exist_ok=True)

        model_path = os.path.join(root_dir, self.model) \
            if not os.path.isabs(self.model) else self.model
        self.detector = VisionDetector(model_path, self.conf)

        self.pub_id = rospy.Publisher("/identificacao",
                                      identificacao, queue_size=10)

        rospy.Subscriber("/scheduler/commands",
                         SchedulerCommand, self._sched_cb, queue_size=30)

        self._active = True

        self.cap = cv2.VideoCapture(self.cam_device)
        if not self.cap.isOpened():
            rospy.logfatal(f"Camera '{self.cam_device}' não abriu.")
            raise RuntimeError("Camera error")

        rospy.loginfo("[vision_identification] pronto. (Q para sair, S para snapshot)")

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Frame perdido!")
                rate.sleep()
                continue

            if self._active:
                ann, classes, cents, angs = self.detector.infer(frame)
                self._publish(classes, cents, angs)

                if self.gui:
                    ann = cv2.resize(ann, None, fx=1.5, fy=1.5,
                                     interpolation=cv2.INTER_LINEAR)
                    cv2.imshow("YOLOv8 - vision_identification", ann)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('s'):
                        self._snapshot(frame)

            else:
                rospy.sleep(0.05)

            rate.sleep()

        self._cleanup()

    def _publish(self,
                 classes: List[str],
                 cents  : List[Tuple[float, float]],
                 angs   : List[Optional[float]]):
        msg = identificacao()
        msg.classes = classes
        msg.centroids = [Point(x=float(cx), y=float(cy)) for cx, cy in cents]
        msg.angles = [a if a is not None else np.nan for a in angs]
        self.pub_id.publish(msg)

    def _snapshot(self, frame):
        name = datetime.now().strftime("foto_%Y%m%d_%H%M%S.jpg")
        cv2.imwrite(os.path.join(self.snap_dir, name), frame)
        rospy.loginfo(f"[vision_identification] snapshot salvo: {name}")

    def _cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def _sched_cb(self, msg: SchedulerCommand):
        if msg.target != "vision":
            return
        payload = msg.payload.strip().lower()
        if payload == "start":
            self._active = True
            rospy.loginfo("[vision_identification] START recebido")
        elif payload == "stop":
            self._active = False
            rospy.loginfo("[vision_identification] STOP recebido")
        elif payload == "capture":
            _, frame = self.cap.read()
            if frame is not None:
                self._snapshot(frame)
        else:
            rospy.logwarn(f"[vision_identification] comando desconhecido: {payload}")

def main():
    try:
        VisionIdentificationNode().spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
