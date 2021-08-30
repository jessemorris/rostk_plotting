import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt



estimated = cv.imread('/home/jesse/Code/src/ros/src/rostk_plotting/records/06:02:2021-12:55:10/1_rgb_prediction.png',0)
stereo = cv.imread('/home/jesse/Code/src/ros/src/rostk_plotting/records/06:02:2021-12:55:10/0.png', 0)

print(estimated.shape)
print(stereo.shape)
print(stereo.min())
print(estimated.dtype)
print(stereo.dtype)


# cv.imshow("Stereo", stereo)
# cv.waitKey(-1)
# hist_mask = cv.calcHist([estimated],[0],None,[256],[0,256])
# plt.imshow(hist_mask, 'gray')

# # stereo = cv.normalize(src=stereo, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
# stereo = cv.bitwise_not(stereo)
# estimated = cv.bitwise_not(estimated)


# # cv.imshow("Stereo", stereo)
# # cv.waitKey(-1)
plt.hist(estimated.ravel(),range(0, 255),[0,256])
plt.hist(stereo.ravel(),range(0, 255),[0,256])
# plt.subplot(121), plt.imshow(hist_estimated, 'Midas')
# plt.subplot(122), plt.imshow(hist_stereo,'Stereo')
# plt.xlim([0,256])

plt.show()