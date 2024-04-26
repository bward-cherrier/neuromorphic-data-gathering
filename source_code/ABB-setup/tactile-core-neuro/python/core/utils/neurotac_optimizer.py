import cv2

def saveFrame(dataPath, video_source):

    cap = cv2.VideoCapture(dataPath + video_source)  
    frameHeight = int(cap.get(4))
    frameWidth = int(cap.get(3))
    radius = 70

    # # Define the codec and create VideoWriter object
    # filename = dataPath + 'masked' + video_source
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    # out = cv2.VideoWriter(filename, fourcc, 30, (frameWidth, frameHeight))  
    
    ret, frame = cap.read()
    cv2.imwrite( dataPath + "maskedTest.jpg", frame);

    # cv2.namedWindow('img')
    # while (cap.isOpened()):
    # 	ret, frame = cap.read()
    #     if ret:
    # 	    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         frame_circle = np.zeros((frameHeight,frameWidth), dtype=frame.dtype)
    #         cv2.circle(frame_circle,(frameWidth/2+5, frameHeight/2+5),radius,1,thickness=-1)
    #         filteredFrame = cv2.bitwise_and(frame,frame,mask=frame_circle)
    #         currFrame = cv2.cvtColor(filteredFrame,cv2.COLOR_GRAY2BGR)
    #         out.write(currFrame)
    #         cv2.imshow('img', filteredFrame)
    #         cv2.waitKey(1)
    #     else:
    #         cap.release()
    #         out.release()

    #         cv2.destroyAllWindows()
    #         break
        # ret, frame = cv2.threshold(frame, threshold ,255,cv2.THRESH_BINARY) # Non-Adaptive Threshhold

    # # Filter noise
    # kernel = np.ones((3,3),np.uint8)
    # frame = cv2.erode(frame,kernel,iterations = 1)
    # frame = cv2.dilate(frame,kernel,iterations = 1)
                 
                 
    # return filteredFrame


def main(*args):

    # Initialise sensor and capture some frames
    dataPath = 'C:\Users\\bw14452\OneDrive - University of Bristol\Analysis\\analysis_tactip-N_Science\Optimization\\'
    vid = 'maskedcalibrationN49L45.avi'
    
    # Write video back to disk with mask
    # writeMaskedVid(dataPath, vid)

    saveFrame(dataPath, vid)

    # raw_input("Press Enter to capture frames (approx 3 secs): ")
    # sensor = TactileSensor(video_source=dataPath + 'masked' + vid, exposure=-6, pin_tracking=False)    
    # try:
    #     frames = sensor.record_frames(100)
    # except:
    #     raise
    # finally:
    	
    #     sensor.close()

    # # Specify parameter bounds and target number of pins 
    # bounds = {'min_threshold' : (0, 300),
    #           'max_threshold' : (0, 300),
    #           'min_area' : (3, 200),
    #           'max_area' : (0, 10),
    #           'min_circularity' : (0.5, 0.9),
    #           'min_convexity' : (0.1, 0.9),
    #           'min_inertia_ratio' : (0.1, 0.9)}

    # n_pins = 49

    # # Run the optimizer    
    # opt_params = optimize(frames, bounds, n_pins)

    # # Try out sensor with optimal parameters
    # raw_input("Press Enter to try out optimized parameters (approx 10 secs): ")    
    # sensor = TactileSensor(video_source=dataPath + 'masked' + vid, exposure=-6, **opt_params)
    # try:
    #     pins = sensor.record_pins(3950)
    #     print("\nShape of captured pin position array: {}".format(pins.shape))
    #     sio.savemat(dataPath + 'pins' + vid[11:-4], {'pins' : pins})

    # except:
    #     raise
    # finally:
    #     sensor.close()


if __name__ == '__main__':
    # logging.basicConfig(stream=sys.stderr)
    # logging.getLogger(__name__).setLevel(logging.DEBUG)
    main()
    