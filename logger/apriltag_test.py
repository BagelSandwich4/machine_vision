import cv2
from pupil_apriltags import Detector

detector = Detector(families="tag25h9")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open camera 0")

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame from camera")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        for r in results:
            print(r.tag_id, r.center, r.corners)
            corners = r.corners.astype(int)
            cv2.polylines(frame, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            cx, cy = r.center.astype(int)
            cv2.putText(frame, str(r.tag_id), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("apriltag", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
