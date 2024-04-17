#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import numpy as np

class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)

        src = np.float32([
            [0, 450],
            [160, 300],
            [480, 300],
            [640, 450]
        ])


        dst = np.float32([
            [160, h],   # 좌하
            [160, 300], # 좌상
            [480, 300], # 우상
            [480,h]     # 우하
        ])

        
        
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )