import pyzed.sl as sl
from datetime import datetime
import os
import time

SAVE_DIR = "/home/aditya/zed_recordings" #Edit this for the other jetson
os.makedirs(SAVE_DIR, exist_ok=True)

while True:
    print("Waiting camera")

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    status = zed.open(init_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera not ready:", status)
        zed.close()
        time.sleep(2)
        continue

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    svo_path = f"{SAVE_DIR}/zed_recording_{timestamp}.svo2"

    recording_params = sl.RecordingParameters(
        svo_path,
        sl.SVO_COMPRESSION_MODE.H264
    )

    status = zed.enable_recording(recording_params)

    if status != sl.ERROR_CODE.SUCCESS:
        print("Failed to start recording:", status)
        zed.close()
        time.sleep(2)
        continue

    print("Recording started:", svo_path)
    print("Press Ctrl+C to stop.")

    try:
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                pass
            else:
                print("Grab failed. Camera may have been unplugged.")
                break
    except KeyboardInterrupt:
        print("Stopping recording")

    zed.disable_recording()
    zed.close()
    break