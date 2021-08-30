import cv2
from matplotlib import pyplot as plt
import numpy as np

img_depth = cv2.imread("/home/jesse/Desktop/Screenshots/lunar_surface_depth_gt.png", cv2.IMREAD_UNCHANGED)

# img_depth = cv2.bitwise_not(img_depth)
img_depth = cv2.resize(img_depth, (640, 480), interpolation = cv2.INTER_AREA)
# img_depth = 1/img_depth
# img_depth = cv2.normalize(src=img_depth, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

# img_invert = img_invert.astype("uint8")
# plt.hist(img_depth.ravel(),255,[0,255]); 
# plt.xlabel("Disparity Values")
# plt.ylabel("No. pixels")
# plt.show()
cv2.imwrite("/home/jesse/Desktop/Screenshots/lunar_surface_depth_gt_invert.png", img_depth)