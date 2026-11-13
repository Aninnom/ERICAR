"""하단 흰 도색 선분 표시·부분 측면선 쌍 정렬. 완전 사각형 요구 없음"""
import cv2
import numpy as np


def white_mask(image):
    if image is None:return None
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([0,0,95],np.uint8),np.array([179,85,255],np.uint8))
    mask[:int(image.shape[0]*.43)]=0
    return cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((2,2),np.uint8))


def white_segments(image):
    mask=white_mask(image)
    if mask is None:return None,[]
    lines=cv2.HoughLinesP(mask,1,np.pi/180,20,minLineLength=25,maxLineGap=12)
    return mask,[] if lines is None else np.asarray(lines).reshape(-1,4).tolist()[:48]


def white_overlay(image):
    mask,lines=white_segments(image)
    if image is None:return None,None,[]
    shown=image.copy()
    shown[mask>0]=(shown[mask>0]*.35+np.array([0,150,0])).astype(np.uint8)
    for x0,y0,x1,y1 in lines:cv2.line(shown,(x0,y0),(x1,y1),(0,255,255),2)
    cv2.putText(shown,f'WHITE SEGMENTS: {len(lines)} (candidates)',(10,24),
                cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),2)
    return shown,mask,lines


def parking_straight_lines(image,config):
    """하단 ROI의 길고 곧은 흰 선. 완전 격자·차선 valid 값 미사용"""
    mask=white_mask(image)
    if mask is None:return None,[]
    h,w=mask.shape
    mask[:int(h*config['white_roi_top'])]=0
    margin=int(w*config['white_roi_side_margin'])
    mask[:,:margin]=0;mask[:,w-margin:]=0
    raw=cv2.HoughLinesP(mask,1,np.pi/180,config['white_hough_votes'],
                        minLineLength=int(w*config['white_min_length_fraction']),maxLineGap=12)
    lines=[]
    if raw is not None:
        for x0,y0,x1,y1 in np.asarray(raw).reshape(-1,4):
            if abs(int(y1)-int(y0))>=h*config['white_min_vertical_fraction']:
                lines.append([int(x0),int(y0),int(x1),int(y1)])
    return mask,lines


def straight_overlay(image,config):
    mask,lines=parking_straight_lines(image,config)
    if image is None:return None,None,[]
    shown=image.copy()
    shown[mask>0]=(shown[mask>0]*.35+np.array([0,150,0])).astype(np.uint8)
    for x0,y0,x1,y1 in lines:cv2.line(shown,(x0,y0),(x1,y1),(0,255,255),2)
    cv2.putText(shown,f'WHITE STRAIGHT: {len(lines)} (parking stop detector)',(10,24),
                cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),2)
    return shown,mask,lines
