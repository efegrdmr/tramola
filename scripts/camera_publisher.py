#!/usr/bin/env python2
import rospy
import cv2
import time
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tramola.msg import Detection 

# Globals
latest_detections = None
last_detection_time = 0.0

def detections_callback(data):
    global latest_detections, last_detection_time
    latest_detections = data.detections
    last_detection_time = time.time()

def create_video_writer(path, fourcc, fps, size):
    vw = cv2.VideoWriter(path, fourcc, fps, size)
    if not vw.isOpened():
        rospy.logerr("Failed to open VideoWriter for file: %s", path)
        return None
    return vw

def safe_close_writer(vw):
    """Release writer and flush OS buffers to minimize corruption risk."""
    try:
        if vw:
            vw.release()
            # Force OS-level buffer flush - reduces corruption risk for already-closed files
            try:
                os.sync()
            except Exception:
                pass
    except Exception as e:
        rospy.logwarn("Error closing VideoWriter: %s", str(e))

def make_segment_path(output_dir, base_name, segment_index):
    timestamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
    # include segment index to guarantee unique filename if multiple segments created same second
    return os.path.join(output_dir, "{}_seg{:03d}_{}".format(timestamp, segment_index, base_name))

def camera_recorder():
    global latest_detections, last_detection_time

    rospy.init_node('camera_recorder_node', anonymous=True)

    # Params
    image_topic = rospy.get_param('~image_topic', 'camera/image_raw')
    annotated_topic = rospy.get_param('~annotated_topic', image_topic + '_annotated')
    detections_topic = rospy.get_param('~detections_topic', '/detections')
    output_dir = rospy.get_param('~output_dir', '/home/tramola/video')
    raw_name = rospy.get_param('~output_video_name_raw', 'output_raw.avi')
    annotated_name_base = rospy.get_param('~output_video_name_annotated', 'output_annotated.avi')
    camera_index = rospy.get_param('~camera_index', 0)
    publish_rate = rospy.get_param('~publish_rate', 30)
    detection_timeout = rospy.get_param('~detection_timeout', 1.0)  # seconds
    record_param_default = rospy.get_param('~record', False)
    segment_seconds = rospy.get_param('~segment_seconds', 30)  # default 5 seconds per segment

    # Ensure output dir exists
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
        except Exception as e:
            rospy.logerr("Could not create output directory '%s': %s", output_dir, str(e))
            return

    # Publishers & subscriber
    bridge = CvBridge()
    raw_pub = rospy.Publisher(image_topic, Image, queue_size=10)
    annotated_pub = rospy.Publisher(annotated_topic, Image, queue_size=10)
    rospy.Subscriber(detections_topic, Detection, detections_callback)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        rospy.logerr("Could not open video device (index=%s)", camera_index)
        return

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
    video_size = (frame_width, frame_height)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps = float(publish_rate)

    rate = rospy.Rate(publish_rate)

    rospy.loginfo("Camera recorder node started. initial record=%s, segment_seconds=%s",
                  record_param_default, segment_seconds)

    # Video writers and segment state
    vw_raw = None
    vw_annot = None
    recording_active = False
    initial_record_request = bool(record_param_default)

    raw_segment_index = 0
    annot_segment_index = 0
    raw_segment_start = None
    annot_segment_start = None
    raw_path = None
    annot_path = None

    try:
        while not rospy.is_shutdown():
            # Dynamic param
            try:
                requested_record = bool(rospy.get_param('~record', initial_record_request))
            except Exception:
                requested_record = initial_record_request

            # Start recording: create raw writer immediately when requested (first segment)
            if requested_record and not recording_active:
                raw_segment_index = 0
                raw_segment_start = time.time()
                raw_path = make_segment_path(output_dir, raw_name, raw_segment_index)
                rospy.loginfo("Recording enabled; opening raw video file: %s", raw_path)
                vw_raw = create_video_writer(raw_path, fourcc, fps, video_size)
                if vw_raw:
                    recording_active = True
                else:
                    rospy.logerr("Raw VideoWriter failed to open; recording not started.")
                    vw_raw = None
                    recording_active = False

            # Stop recording: close any open writers
            if (not requested_record) and recording_active:
                rospy.loginfo("Recording disabled; closing video files.")
                safe_close_writer(vw_raw)
                safe_close_writer(vw_annot)
                vw_raw = None
                vw_annot = None
                recording_active = False
                raw_segment_index = 0
                annot_segment_index = 0
                raw_segment_start = None
                annot_segment_start = None
                raw_path = None
                annot_path = None

            ret, frame = cap.read()
            if not ret:
                rospy.logwarn_throttle(5, "Camera read returned no frame.")
                rate.sleep()
                continue

            # Publish raw frame always
            try:
                raw_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                raw_pub.publish(raw_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge error (raw publish): %s", str(e))

            # Prepare annotated frame if detections are recent
            current_time = time.time()
            detections_valid = (latest_detections is not None and (current_time - last_detection_time) <= detection_timeout)

            if detections_valid:
                annotated_frame = frame.copy()
                for detection in latest_detections:
                    x_center = int(detection.x_center * frame_width)
                    y_center = int(detection.y_center * frame_height)
                    width = int(detection.width * frame_width)
                    height = int(detection.height * frame_height)

                    x1 = max(0, int(x_center - width / 2))
                    y1 = max(0, int(y_center - height / 2))
                    x2 = min(frame_width - 1, int(x_center + width / 2))
                    y2 = min(frame_height - 1, int(y_center + height / 2))

                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = "ID: {} Conf: {:.2f}".format(detection.class_id, detection.confidence)
                    cv2.putText(annotated_frame, label, (x1, max(0, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                timestamp_text = "Time: {}".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
                cv2.putText(annotated_frame, timestamp_text, (10, frame_height - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # If recording is active and annotated writer not created yet, create it now (first segment)
                if recording_active and vw_annot is None:
                    annot_segment_index = 0
                    annot_segment_start = time.time()
                    annot_path = make_segment_path(output_dir, annotated_name_base, annot_segment_index)
                    rospy.loginfo("First detection seen; creating annotated video file: %s", annot_path)
                    vw_annot = create_video_writer(annot_path, fourcc, fps, video_size)
                    if vw_annot is None:
                        rospy.logerr("Annotated VideoWriter failed to open; annotated frames will not be saved.")
            else:
                annotated_frame = frame

            # Publish annotated topic always (annotated when detections exist)
            try:
                annotated_msg = bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                annotated_pub.publish(annotated_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge error (annotated publish): %s", str(e))

            # Handle segment rotation for raw (if recording)
            if recording_active and vw_raw:
                # Write raw frame
                try:
                    vw_raw.write(frame)
                except Exception as e:
                    rospy.logwarn("Error writing raw frame to video: %s", str(e))

                # rotate raw segment if elapsed
                if raw_segment_start and (time.time() - raw_segment_start >= float(segment_seconds)):
                    rospy.loginfo("Rotating raw segment (index %d)...", raw_segment_index)
                    safe_close_writer(vw_raw)
                    raw_segment_index += 1
                    raw_segment_start = time.time()
                    raw_path = make_segment_path(output_dir, raw_name, raw_segment_index)
                    rospy.loginfo("Opening new raw segment: %s", raw_path)
                    vw_raw = create_video_writer(raw_path, fourcc, fps, video_size)
                    if vw_raw is None:
                        rospy.logerr("Failed to open new raw segment writer; stopping raw writes.")
                        vw_raw = None

            # Handle annotated write & segmentation: only write when detections_valid and vw_annot exists
            if detections_valid and vw_annot:
                try:
                    vw_annot.write(annotated_frame)
                except Exception as e:
                    rospy.logwarn("Error writing annotated frame to video: %s", str(e))

                # rotate annotated segment if elapsed
                if annot_segment_start and (time.time() - annot_segment_start >= float(segment_seconds)):
                    rospy.loginfo("Rotating annotated segment (index %d)...", annot_segment_index)
                    safe_close_writer(vw_annot)
                    annot_segment_index += 1
                    annot_segment_start = time.time()
                    annot_path = make_segment_path(output_dir, annotated_name_base, annot_segment_index)
                    rospy.loginfo("Opening new annotated segment: %s", annot_path)
                    vw_annot = create_video_writer(annot_path, fourcc, fps, video_size)
                    if vw_annot is None:
                        rospy.logerr("Failed to open new annotated segment writer; stopping annotated writes.")
                        vw_annot = None

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            cap.release()
        except Exception:
            pass
        safe_close_writer(vw_raw)
        safe_close_writer(vw_annot)
        cv2.destroyAllWindows()
        rospy.loginfo("Camera recorder node exiting.")

if __name__ == '__main__':
    camera_recorder()
