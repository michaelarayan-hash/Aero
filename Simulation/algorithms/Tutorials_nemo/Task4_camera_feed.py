from gz_cam import SimCamera                                                                                                                                                            
import time                                                                              

                                                                                                                                                       
cam = SimCamera(world='tag_demo')                                                                                                                                                                       
if not cam.wait_for_frame():                                                                                                                                                            
    print("[ERROR] No frames — is the sim running?")
else:                                                                                                                                                                                   
    ok, frame = cam.read()
    print(f"ok={ok}  shape={frame.shape}")           
    time.sleep(0.5)                                                                                                                                   
                                            