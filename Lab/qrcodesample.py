import cv2
import numpy as np

# create detector
detector = cv2.QRCodeDetector()

# open default webcam
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # detect and decode QR code
    data, points, _ = detector.detectAndDecode(frame)

    if points is not None and data:
        print("QR detected:", data)

        # draw box around QR
        points = points[0].astype(int)
        for i in range(len(points)):
            pt1 = tuple(points[i])
            pt2 = tuple(points[(i + 1) % len(points)])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # show decoded text on frame
        cv2.putText(
            frame,
            data,
            (points[0][0], points[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    # show webcam feed
    cv2.imshow("QR Detection - Webcam", frame)

    # press q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
