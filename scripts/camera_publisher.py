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
    filename = "{}_seg{:03d}_{}".format(timestamp, segment_index, base_name)
    # Ensure it has .mp4 extension
    if not filename.lower().endswith('.mp4'):
        filename = os.path.splitext(filename)[0] + '.mp4'
    return os.path.join(output_dir, filename)

def camera_recorder():
    global latest_detections, last_detection_time

    rospy.init_node('camera_recorder_node', anonymous=True)

    # Params
    image_topic = rospy.get_param('~image_topic', 'camera/image_raw')
    annotated_topic = rospy.get_param('~annotated_topic', image_topic + '_annotated')
    detections_topic = rospy.get_param('~detections_topic', '/detections')
    output_dir = rospy.get_param('~output_dir', '/home/tramola/video')
    output_video_name = rospy.get_param('~output_video_name', 'output_video.mp4')
    camera_index = rospy.get_param('~camera_index', 0)
    publish_rate = rospy.get_param('~publish_rate', 30)
    detection_timeout = rospy.get_param('~detection_timeout', 1.0)  # seconds
    record_param_default = rospy.get_param('~record', False)
    segment_seconds = rospy.get_param('~segment_seconds', 30)  # default 30 seconds per segment

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
    
    # Use MP4 codec
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Using MP4V codec for MP4 format
    fps = float(publish_rate)

    rate = rospy.Rate(publish_rate)

    rospy.loginfo("Camera recorder node started. initial record=%s, segment_seconds=%s",
                  record_param_default, segment_seconds)

    # Video writer and segment state
    video_writer = None
    recording_active = False
    initial_record_request = bool(record_param_default)

    segment_index = 0
    segment_start = None
    video_path = None
    
    # For timestamp display
    start_time = time.time()

    try:
        while not rospy.is_shutdown():
            # Dynamic param
            try:
                requested_record = bool(rospy.get_param('~record', initial_record_request))
            except Exception:
                requested_record = initial_record_request

            # Start recording: create writer immediately when requested (first segment)
            if requested_record and not recording_active:
                segment_index = 0
                segment_start = time.time()
                start_time = time.time()  # Reset start time when recording begins
                video_path = make_segment_path(output_dir, output_video_name, segment_index)
                rospy.loginfo("Recording enabled; opening video file: %s", video_path)
                video_writer = create_video_writer(video_path, fourcc, fps, video_size)
                if video_writer:
                    recording_active = True
                else:
                    rospy.logerr("VideoWriter failed to open; recording not started.")
                    video_writer = None
                    recording_active = False

            # Stop recording: close any open writer
            if (not requested_record) and recording_active:
                rospy.loginfo("Recording disabled; closing video file.")
                safe_close_writer(video_writer)
                video_writer = None
                recording_active = False
                segment_index = 0
                segment_start = None
                video_path = None

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
            detections_valid = (latest_detections is not None and 
                               (current_time - last_detection_time) <= detection_timeout)

            # Frame to record - either annotated or raw based on detection availability
            frame_to_record = frame.copy()  # Create a copy to always add timestamp

            # Get current ROS time with only seconds
            ros_time = rospy.Time.now()
            seconds_only = ros_time.secs
            timestamp_text = "{}".format(seconds_only)
            
            # Add timestamp to the frame_to_record
            cv2.putText(frame_to_record, timestamp_text, (10, frame_height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if detections_valid:
                # If detections are valid, add bounding boxes to the frame_to_record
                for detection in latest_detections:
                    x_center = int(detection.x_center * frame_width)
                    y_center = int(detection.y_center * frame_height)
                    width = int(detection.width * frame_width)
                    height = int(detection.height * frame_height)

                    x1 = max(0, int(x_center - width / 2))
                    y1 = max(0, int(y_center - height / 2))
                    x2 = min(frame_width - 1, int(x_center + width / 2))
                    y2 = min(frame_height - 1, int(y_center + height / 2))

                    cv2.rectangle(frame_to_record, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = "ID: {} Conf: {:.2f}".format(detection.class_id, detection.confidence)
                    cv2.putText(frame_to_record, label, (x1, max(0, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Copy to annotated_frame for publishing
                annotated_frame = frame_to_record.copy()
            else:
                # Just timestamp for annotated frame when no detections
                annotated_frame = frame.copy()
                cv2.putText(annotated_frame, timestamp_text, (10, frame_height - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Publish annotated topic always
            try:
                annotated_msg = bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                annotated_pub.publish(annotated_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge error (annotated publish): %s", str(e))

            # Handle recording and segment rotation
            if recording_active and video_writer:
                # Write frame (either annotated or raw depending on detection availability)
                try:
                    video_writer.write(frame_to_record)
                except Exception as e:
                    rospy.logwarn("Error writing frame to video: %s", str(e))

                # rotate segment if elapsed
                if segment_start and (time.time() - segment_start >= float(segment_seconds)):
                    rospy.loginfo("Rotating video segment (index %d)...", segment_index)
                    safe_close_writer(video_writer)
                    segment_index += 1
                    segment_start = time.time()
                    video_path = make_segment_path(output_dir, output_video_name, segment_index)
                    rospy.loginfo("Opening new video segment: %s", video_path)
                    video_writer = create_video_writer(video_path, fourcc, fps, video_size)
                    if video_writer is None:
                        rospy.logerr("Failed to open new video segment writer; stopping recording.")
                        video_writer = None
                        recording_active = False

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            cap.release()
        except Exception:
            pass
        safe_close_writer(video_writer)
        cv2.destroyAllWindows()
        rospy.loginfo("Camera recorder node exiting.")

if __name__ == '__main__':
    camera_recorder()