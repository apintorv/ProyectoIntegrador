import cv2 as cv
import os

delay = 150
disp_width = 640
disp_height = 480
flip = 0
cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'

def save_frame(frame, path, count):
    cv.imwrite(path+'fire%d.jpg'% count, frame) # nombre de la clase 


if __name__ == "__main__":
    count = 0
    camera = cv.VideoCapture(cam_port)  
    while True: 
        ret, frame = camera.read()
        save_frame(frame,"/home/puzzlebot/Documents/Challenge/images", count) #path carpeta images
        count += 1
        if cv.waitKey(delay) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()
