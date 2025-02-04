#!/usr/bin/env python3

'''
git clone https://github.com/Tencent/ncnn.git
cd ncnn/tools/darknet2onnx
python3 darknet2onnx.py --cfg yolov4.cfg --weights yolov4.weights --output yolov4.onnx
#onxx oluştu
cd ..
/tramola/src/tensorrt/bin/trtexec --onnx=yolov4.onnx --saveEngine=yolov4.trt --fp16
#tensorrt tam konum bakilmali
#trt oluştu
'''

import os
import cv2
import rospy
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from tramola.msg import Detection, DetectionList

class YOLOv4TensorRT:
    def __init__(self):
        rospy.init_node('yolo_detection_node', anonymous=True)
        self.publisher = rospy.Publisher('yolo_detections', DetectionList, queue_size=10)
        self.rate = rospy.Rate(10)

        #TRT PATH DEGISTIR
        self.MODEL_PATH = os.getenv('MODEL_PATH', 'path_to_yolov4.trt')
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        rospy.loginfo(f"Loading TensorRT model from {self.MODEL_PATH}")
        with open(self.MODEL_PATH, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings = self.allocate_buffers()

        self.cap = cv2.VideoCapture(0)

    def allocate_buffers(self):
        """Allocates memory for TensorRT inference"""
        inputs, outputs, bindings = [], [], []
        for i in range(self.engine.num_bindings):
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            shape = self.engine.get_binding_shape(i)
            size = trt.volume(shape) * np.dtype(dtype).itemsize
            device_mem = cuda.mem_alloc(size)

            if self.engine.binding_is_input(i):
                inputs.append({"shape": shape, "dtype": dtype, "device": device_mem})
            else:
                outputs.append({"shape": shape, "dtype": dtype, "device": device_mem})

            bindings.append(int(device_mem))

        return inputs, outputs, bindings

    def detect_objects(self, frame):
        """Runs YOLOv4 inference using TensorRT"""
        input_size = (416, 416)
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, input_size, swapRB=True, crop=False)

        cuda.memcpy_htod(self.inputs[0]['device'], blob.ravel())

        # Run inference
        self.context.execute_v2(self.bindings)

        # Retrieve and process output
        output = np.empty(self.outputs[0]['shape'], dtype=self.outputs[0]['dtype'])
        cuda.memcpy_dtoh(output, self.outputs[0]['device'])
        return self.postprocess(output, frame.shape[:2])

    def postprocess(self, output, img_shape):
        """Decodes YOLOv4 output into detection format"""
        boxes, confidences, class_ids = [], [], []
        for detection in output:
            x, y, w, h, conf, class_id = detection[:6]  # Assumed YOLO format
            if conf > 0.5:
                boxes.append([x, y, w, h])
                confidences.append(conf)
                class_ids.append(int(class_id))

        # Apply Non-Maximum Suppression (NMS)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        detections = []
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            detections.append([x, y, w, h, class_ids[i], confidences[i]])

        return detections

    def run(self):
        """Main loop for capturing, detecting, and publishing detections"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                continue

            detections = self.detect_objects(frame)

            height, width, _ = frame.shape
            detection_list = DetectionList()
            for x, y, w, h, class_id, confidence in detections:
                detection = Detection()
                detection.x_center = (x + w / 2.0) / width
                detection.y_center = (y + h / 2.0) / height
                detection.width = w / width
                detection.height = h / height
                detection.confidence = confidence
                detection.class_id = class_id
                detection_list.detections.append(detection)

            self.publisher.publish(detection_list)
            rospy.loginfo("Published {} detections.".format(len(detection_list.detections)))
            self.rate.sleep()

        self.cap.release()

if __name__ == '__main__':
    try:
        detector = YOLOv4TensorRT()
        detector.run()
    except rospy.ROSInterruptException:
        pass
