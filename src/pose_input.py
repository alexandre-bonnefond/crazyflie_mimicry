import mediapipe as mp
import cv2 
from multiprocessing import Array
import utils.camera_calculation as camcalc

model_path = '/home/alexandre/Dev/crazyflie_mimicry/pose_landmarker_lite.task'

# wrists_pos = Array('d', [-1, -1])
left_wrist_pos = Array('d', [-1, -1])
right_wrist_pos = Array('d', [-1, -1])


BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

# Create a pose landmarker instance with the live stream mode:
def print_result(result: PoseLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    try:
        left_wrist = result.pose_landmarks[0][15]
        right_wrist = result.pose_landmarks[0][16]
        # wrists_pos[0] = left_wrist.y if left_wrist.presence > 0.85 else -1
        # wrists_pos[1] = right_wrist.y if right_wrist.presence > 0.85 else -1
        if left_wrist.presence > 0.70:     
            left_wrist_pos[0], left_wrist_pos[1] = left_wrist.x, left_wrist.y
        else:
            left_wrist_pos[0], left_wrist_pos[1] = -1, -1

        if right_wrist.presence > 0.70:     
            right_wrist_pos[0], right_wrist_pos[1] = right_wrist.x, right_wrist.y
        else:
            right_wrist_pos[0], right_wrist_pos[1] = -1, -1


        # print('left wrist is at x={} and y={} with confidence = {}'.format(left_wrist.x, left_wrist.y, left_wrist.presence))
        # print('right wrist is at x={} and y={} with confidence = {}'.format(right_wrist.x, right_wrist.y, right_wrist.presence))
    except Exception as e:
        right_wrist_pos[0], right_wrist_pos[1] = -1, -1
        left_wrist_pos[0], left_wrist_pos[1] = -1, -1
        # print(e)
    # print('left wrist is at x={} and y={}'.format(left_wrist_pos[0], left_wrist_pos[1]))
    # print('right wrist is at x={} and y={}'.format(right_wrist_pos[0], right_wrist_pos[1]))




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
            # avg = camcalc.compute_prefered_direction(left_wrist_pos, right_wrist_pos, 1.4*0.35)
            # eq = camcalc.compute_equilibrium_distance(left_wrist_pos, right_wrist_pos, 1)
            # print(eq)
            # camcalc.compute_yaw(left_wrist_pos, right_wrist_pos, 0)
        
        vid.release() 
        cv2.destroyAllWindows()