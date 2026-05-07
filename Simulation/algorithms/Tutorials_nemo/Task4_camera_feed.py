from gz_cam import SimCamera                                                                                                                                                            
import time                                                                              
import cv2
                                                                                                                                                       
cam = SimCamera(world='aruco', vehicle='x500_mono_cam_down')                                                                                                                                                                       
if not cam.wait_for_frame():                                                                                                                                                            
    print("[ERROR] No frames — is the sim running?")
else:                        
    while True:
        ret, frame = cam.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            break
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) == 27:  # ESC key
            break                