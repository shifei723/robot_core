"""Ensure the deployed ONNX model runs using the new module's local assets."""
import unittest
from pathlib import Path
import cv2
import numpy as np
from yolo_semantics import YOLOSemantics


class ModelDeploymentTest(unittest.TestCase):
    def test_empty_scene_inference(self):
        cv2.setNumThreads(2)
        path=Path(__file__).resolve().parents[1]/'models/yolox_tiny.onnx'
        self.assertTrue(path.is_file(), 'Download model with scripts/download_yolo_model.sh')
        model=YOLOSemantics(path)
        detections=model.infer(np.full((480,640,3),114,np.uint8))
        self.assertEqual(detections, [], 'Empty synthetic image should not create semantic objects')


if __name__=='__main__': unittest.main()
