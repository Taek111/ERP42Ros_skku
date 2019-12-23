import cv2
import sys
import math
import cv2 as cv
import numpy as np
 
cap = cv2.VideoCapture("images/video2.mp4")

 
while (True):
    ret, img = cap.read()
    img = cv2.resize(img,(640,360))
    height, width = img.shape[:2]
    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (3,3), 0)

    canny_img = cv2.Canny(img, 50, 200)

    vertices = np.array([[(50,height-70),(width/2-180, height/2-100), (width/2+180, height/2-100), (width-50,height-70)]],
                        dtype=np.int32)
    ROI_img = region_of_interest(canny_img, vertices,(0,0,255))
#     cv2.imshow('d', canny_img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     cv2.waitKey(1)
    line_arr = cv2.HoughLinesP(ROI_img, 1, 1 * np.pi/180, 50, np.array([]), 50, 10) # 허프 변환 
    line_arr = np.squeeze(line_arr)
    temp = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(temp, line_arr[:,None])

    slope_degree = np.arctan2(line_arr[:, 1] - line_arr[:, 3], line_arr[:,0] - line_arr[:, 2]) * 180 / np.pi
  
    # 수평 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)<140]
    slope_degree = slope_degree[np.abs(slope_degree)<140]
    # 수직 기울기 제한
    line_arr = line_arr[np.abs(slope_degree)>95]
    slope_degree = slope_degree[np.abs(slope_degree)>95]
#     print(slope_degree)
#     break
    L_lines, R_lines = line_arr[(slope_degree>0),:], line_arr[(slope_degree<0),:]
    
    L_lines, R_lines = L_lines[:,None], R_lines[:,None]

    # 직선 그리기
    draw_lines(temp, L_lines)
    draw_lines(temp, R_lines)
    
    result = weighted_img(temp, img) # 원본 이미지에 검출된 선 overlap
    cv2.imshow('result',result) # 결과 이미지 출력
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    elif cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite('capture.png', img)
        print("captured")
 
cap.release()
cv2.destroyAllWindows()