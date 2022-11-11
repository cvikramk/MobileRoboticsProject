import pupil_apriltags
import cv2
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")

image = cv2.imread('./tools/Examples-of-different-AprilTag-families.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(gray.shape)
#options = pupil_apriltags.DetectorOptions(families="tag16h5")

detector = pupil_apriltags.Detector(families="tag36h11")
results = detector.detect(gray)
print(results[0].corners)

# Draw rectangle around tag
#print(results.tag_family, results.tag_id, results.center, results.corners)
for i in results:
    print(i.tag_family, i.tag_id, i.center, i.corners)
    print("Corners ",int(i.corners[0][0]))
    image = cv2.rectangle(image, (int(i.corners[0][0]), int(i.corners[0][1])),(int(i.corners[2][0]),int(i.corners[2][1])), (255, 255, 0), 10)

# cv2.imshow('image', image)
# cv2.waitKey(0)
# # plt.imshow(gray)
# # plt.show()
# print(len(results))

# detecting the tag in live video
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray, estimate_tag_pose=True, camera_params=(500, 500, 320, 240), tag_size=0.06)
    for i in results:
        print(i.tag_family, i.tag_id, i.center, i.corners)
        print("Corners ",int(i.corners[0][0]))
        frame = cv2.rectangle(frame, (int(i.corners[0][0]), int(i.corners[0][1])),(int(i.corners[2][0]),int(i.corners[2][1])), (255, 255, 0), 10)
        frame = cv2.putText(frame, str(i.tag_id), (int(i.corners[0][0]), int(i.corners[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        frame = cv2.putText(frame, str(i.pose_R), (int(i.corners[0][0]), int(i.corners[0][1]) + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        frame = cv2.putText(frame, str(i.pose_t), (int(i.corners[0][0]), int(i.corners[0][1]) + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

