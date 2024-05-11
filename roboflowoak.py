from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

if __name__ == '__main__':
    # instantiating an object (rf) with the RoboflowOak module
    rf = RoboflowOak(model="redsquare-gwdyn", confidence=0.50,
    overlap=0.5, version="1",
    api_key="N5Xs9o02pFDsaVc5pjcd", rgb=True, depth=False,
    device=None, device_name="NGCP", blocking=True)
    while True:
        t0 = time.time()
        result, frame, raw_frame, depth = rf.detect()    # verify size of capture frame is 640x640
        predictions = result["predictions"]
        
        t = time.time()-t0
        print("INFERENCE TIME IN MS ", 1/t)
        
        for p in predictions:
            x = p.json()['x']
            y = p.json()['y']
            width = p.json()['width']
            height = p.json()['height']
            print("Center X:", x, "Center Y:", y, "Width:", width, "Height:", height)
        
        cv2.imshow("frame", frame) 
        if cv2.waitKey(1) == ord('q'):
            break
