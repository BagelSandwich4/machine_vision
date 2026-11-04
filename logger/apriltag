import apriltag
import cv2

detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
results = detector.detect(gray)
for r in results:
    print(r.tag_id, r.center, r.corners)