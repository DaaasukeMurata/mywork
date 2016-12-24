#coding: utf-8
import cv2
import numpy as np

#サイズの同じ画像を２枚読み込んでおく
imgA = cv2.imread('A.jpg', 1)
imgB = cv2.imread('B.jpg', 1)

#画像の大きさと次元が同じあることを確認しておく
print imgA.shape
print imgB.shape

#addWeightedで混ぜていく
#Aの重み0.3 Bの重み0.7 ガンマ2.2
img_03_07 = cv2.addWeighted(imgA, 0.3, imgB, 0.7, 2.2)

#Aの重み0.5 Bの重み0.5 ガンマ2.2
img_05_05 = cv2.addWeighted(imgA, 0.5, imgB, 0.5, 2.2)

#Aの重み0.7 Bの重み0.3 ガンマ2.2
img_07_03 = cv2.addWeighted(imgA, 0.7, imgB, 0.3, 2.2)

cv2.imwrite('img_0.5_0.5.jpg', img_05_05)
cv2.imwrite('img_0.3_0.7.jpg', img_03_07)
cv2.imwrite('img_0.7_0.3.jpg', img_07_03)
