import cv2
import time

def video_capture(camera_id, output_filename, fps=30, width=640, height=480):
    """
    Capture video from a camera and save it to a file.

    :param camera_id: ID of the camera to use (default is 0). 
    :param output_filename: Name of the output video file. 
    :param fps: Frames per second for the video.
    :param width: Width of the video frames. 
    :param height: Height of the video frames. 
    """
    cap = cv2.VideoCapture(camera_id)  
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height) 
    cap.set(cv2.CAP_PROP_FPS, fps)   

    if not cap.isOpened():
        print("Cannot open camera")  
        exit(-1)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Define the codec
    out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height)) 

    start_time = time.time()
    print("Start recording...")  

    while cap.isOpened():
        ret, frame = cap.read()  
        if not ret:
            print("Can't get frame, stop recording")  
            break

        out.write(frame) 

        if time.time() - start_time > 120:
            print("Recording finished") 
            break

    cap.release()  
    out.release()  
    cv2.destroyAllWindows() 

if __name__ == "__main__":
    camera_id = 0 # v4l2-ctl 可使用此命令列出设备
    output_filename = '../dataset/raw/output.avi'
    fps = 30 
    width = 640 
    height = 480 

    video_capture(camera_id, output_filename, fps, width, height)