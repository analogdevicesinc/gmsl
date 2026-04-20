from  saleae_i2c_capture import GmslCaptureManager
import time
import os
import logging
import cv2 as cv
from cv2_enumerate_cameras import enumerate_cameras
import scipy.stats as scistats
from config import *
from ps3005d import PS3005D

# Runs a repeat system start and log test.
# Automates power supply and Saleae logic analyzer capture
# Connect logic analyser to I2C lines in the system

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', force=True)

def check_frame(frame):
    logging.debug("Check photo content for good image")
    
    #find average hsv value for region of interest
    roi_rgb = frame[roi_pts[0][1]:roi_pts[1][1], roi_pts[0][0]:roi_pts[1][0]]
    roi_hsv = cv.cvtColor(roi_rgb, cv.COLOR_BGR2HSV)
    roi_average = cv.mean(roi_hsv)

    #Hue is circular so need to use circular mean                       
    roi_average = list(roi_average)
    roi_average[0] = scistats.circmean(roi_hsv.flatten(), high=180, low=0)
    
    logging.debug("Average HSV for roi: %s" % str(roi_average))
    
    pass_image = False
    
    #check if average hsv is within limits
    #TODO - Test hue error with rotational distance, not min max
    sat_min = hsv_target[1] - hsv_error[1]
    sat_max = hsv_target[1] + hsv_error[1]
    val_min = hsv_target[2] - hsv_error[2]
    val_max = hsv_target[2] + hsv_error[2]
    hue_max_error = hsv_error[0]
    
    hue = roi_average[0]
    sat = roi_average[1]
    val = roi_average[2]
    
    hue_error = hue - hsv_target[0]
    if hue_error > 90:
        hue_error = 180 - hue_error
    elif hue_error < -90:
        hue_error = 180 + hue_error
    hue_abs_error = abs(hue_error)
                        
    if val > val_min and val < val_max and sat > sat_min and sat < sat_max and hue_abs_error < hue_max_error:
        logging.debug("Good image")
        pass_image = True
    else:
        logging.debug("Bad image")
        
    return {'pass': pass_image, 'roi_average': roi_average}


def show_frame(frame, pass_image, roi_average):
    #put the roi on the frame
    cv.rectangle(frame, roi_pts[0], roi_pts[1], (0, 255, 0), 2)
    
    #print the average value on the frame
    cv.putText(frame, str(roi_average), (100, 50), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
    
    #print the pass/fail on the frame
    cv.putText(frame, "PASS" if pass_image else "FAIL", (100, 100), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if pass_image else (0, 0, 255), 2)
                            
    #show image
    cv.imshow('image', frame)
    
    #wait for key press
    key = cv.waitKey(10)

    #if key is escape or q for quit then return false
    if key == 27 or key == ord('q'):
        return False
    return True


def main():
    cap_manager = GmslCaptureManager(capture_time=capture_time)
    
    #Configure power supply
    ps3005d = None
    if power_supply == "PS3005D":
        try:
            ps3005d = PS3005D(PS3005D_PORT)
            time.sleep(1)
            ps3005d.turn_off()
            time.sleep(1)
            ps3005d.set_voltage(12.0)
            time.sleep(1)
            ps3005d.set_current(4.0)
        except:
            logging.error("Failed to connect to PS3005D")
            ps3005d = None
        
  
    cam_index = -1
    #get list of cameras and pick one with preference for not default inbuilt camera
    for camera_info in enumerate_cameras(cv.CAP_DSHOW):
        print(f'{camera_info.index}: {camera_info.name}')
        if cam_name in camera_info.name:
            cam_index = camera_info.index
            logging.info("New camera index is %d" % cam_index)
    
    if cam_index == -1:
        logging.error("Camera string:%s not found" % cam_name)
        return

    #Open camera at the index found   
    cap = cv.VideoCapture(cam_index, cv.CAP_DSHOW)

    #set resolution
    hres = 1024
    vres = 720
    logging.info("Camera resolution set is %dx%d" % (hres, vres))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, hres)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, vres)
    
    #get actual resolution
    hres = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    vres = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    
    logging.info("Camera resolution is %dx%d" % (hres, vres))
        
    date_time = time.strftime("%Y-%m-%d-%H-%M-%S")
    output_directory = os.path.join(os.getcwd(), 'output', date_time)
    
    #create output directory, checkig if it already exists
    try:
        os.makedirs(output_directory, exist_ok=False)
    except FileExistsError:
        logging.error("Output directory:%s already exists" % output_directory)
        return
    
    try:
        results_path = os.path.join(output_directory, 'results.csv')
        with open (results_path, 'w') as results_file:
            if WRITE_DEBUG_VIDEO:
                mp4_path = os.path.join(output_directory, 'output.mp4')
                fourcc = cv.VideoWriter_fourcc(*'mp4v')
                video = cv.VideoWriter(mp4_path, fourcc, 20, (hres, vres))

                # avi_path = os.path.join(output_directory, 'output.avi')
                # fourcc = cv.VideoWriter_fourcc(*'XVID')
                # video = cv.VideoWriter(avi_path, fourcc, 20, (hres, vres))
            for index in range(captures):
                filepath = os.path.join(output_directory, 'capture_%d.sal' % index)
                
                logging.info("Power supply off")
                try:
                    ps3005d.turn_off()
                except:
                    pass
                        
                if cap_manager.trigger_capture(filepath):
                    logging.debug("Wait 1s")
                    time.sleep(1)

                    logging.info("Power supply on")
                    try:
                        ps3005d.turn_on()
                    except:
                        pass
                    
                    #Capture does not start until first falling edge
                    while(not cap_manager.capture_done()):
                        time.sleep(1)
                    
                    logging.debug("Take camera photo")
                    ret, frame = cap.read()

                    if not ret:
                        logging.error("Failed to take photo")
                        break
                    
                    logging.debug("Save image")
                    image_path = os.path.join(output_directory, 'image_%d.jpg' % index)
                    cv.imwrite(image_path, frame)
                    
                    #write frame to video
                    if WRITE_DEBUG_VIDEO:
                        logging.debug("Write to video")
                        video.write(frame)
                    
                    check_frame_results = check_frame(frame)
                    
                    roi_average = check_frame_results['roi_average']
                    pass_image = check_frame_results['pass']

                    #write results to file
                    results_file.write("%d,%s,%s\n" % (index, str(roi_average), "PASS" if pass_image else "FAIL"))
                    logging.debug("Results: %s" % str(check_frame_results))
                    
                    if SHOW_DEBUG_FRAME:
                        #Show frame and exit if user reqests with escape or q
                        if not show_frame(frame, pass_image, roi_average):
                            try:
                                video.release()
                            except:
                                pass
                            break
                else:
                    raise Exception("Capture trigger failed")
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt")
        try:
            video.release()
        except:
            pass
        
    cap_manager.request_exit()
    
if __name__ == '__main__':
    main()