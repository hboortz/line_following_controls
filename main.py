import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    cv2.imshow('frame',frame)

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_bound = np.array([0,0,0])
    upper_bound = np.array([255,255,100])

    # Threshold the HSV image to get the black line
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    middle = mask.shape[0]/2

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask=mask)


    contours, heirarchy = cv2.findContours(mask, 1, 2)
    cnt = contours[0]
    momt = cv2.moments(cnt)
    print momt

    try:
        cx = int(momt['m10']/momt['m00'])
        cy = int(momt['m01']/momt['m00'])
        print cx, cy
    except ZeroDivisionError:
        pass
    raw_input()
    cv2.imshow('mask',mask)


    # detector = cv2.SimpleBlobDetector()
    
    # # Detect blobs
    # keypoints = detector.detect(im)
    
    # # Draw detected blobs as red circles.
    # # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    # im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Keypoints", im_with_keypoints)

    # cv2.imshow('frame',frame)
    # cv2.imshow('mask',mask)
    # cv2.imshow('res',res)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()