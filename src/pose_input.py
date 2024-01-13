import mediapipe as mp
import time
import cv2 
from multiprocessing import Array

model_path = '/home/alexandre/Dev/crazyflie_mimicry/pose_landmarker_lite.task'

wrists_pos = Array('d', [-1, -1])

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

# Create a pose landmarker instance with the live stream mode:
def print_result(result: PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    left_wrist = result.pose_landmarks[0][15]
    right_wrist = result.pose_landmarks[0][16]
    wrists_pos[0] = left_wrist.y if left_wrist.presence > 0.85 else -1
    wrists_pos[1] = right_wrist.y if right_wrist.presence > 0.85 else -1
    # print('left wrist is at y={} with confidence = {}'.format(left_wrist.y, left_wrist.presence))
    # print('right wrist is at y={} with confidence = {}'.format(right_wrist.y, right_wrist.presence))

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result)

 

def detect_wrists_pos():

    vid = cv2.VideoCapture(0)
    frame_id = 0

    with PoseLandmarker.create_from_options(options) as landmarker:
        while True:
            ret, frame = vid.read() 
            cv2.imshow('frame', frame) 

            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break
            
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            landmarker.detect_async(mp_image, frame_id)
            frame_id += 1
            # print(wrists_pos[0])
            # time.sleep(0.1)
        
        vid.release() 
        cv2.destroyAllWindows()