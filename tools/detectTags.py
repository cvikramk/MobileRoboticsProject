import apriltag
import cv2
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("TkAgg")
image = cv2.imread('test_tag_full.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(gray.shape)
options = apriltag.DetectorOptions(families="tag16h5")
detector = apriltag.Detector()
results = detector.detect(gray)
plt.imshow(gray)
plt.show()
print(len(results))

