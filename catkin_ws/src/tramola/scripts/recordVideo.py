import cv2
import time

def record_video(output_file, duration, fps=20.0):
    # Open the default camera
    cap = cv2.VideoCapture(0)

    # Get the native resolution of the camera
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_size = (frame_width, frame_height)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, fps, frame_size)

    # Calculate the number of frames to capture
    num_frames = int(duration * fps)

    for _ in range(num_frames):
        ret, frame = cap.read()
        if ret:
            out.write(frame)
        else:
            break

    # Release everything if job is finished
    cap.release()
    out.release()

if __name__ == "__main__":
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    output_file = f'output_{timestamp}.avi'
    duration = 10  # Duration in seconds
    record_video(output_file, duration)
