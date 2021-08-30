import cv2

img = cv2.imread("/home/jesse/Code/src/ros/src/rostk_plotting/records/06:02:2021-12:55:10/5.png", cv2.IMREAD_UNCHANGED)

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
cv2.imwrite("/home/jesse/Code/src/ros/src/rostk_plotting/records/06:02:2021-12:55:10/5_rgb.png", img_rgb)