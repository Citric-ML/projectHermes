from ..sensors.camera import capture_image
import cv2

frame_path = capture_image()

frame = cv2.imread(str(frame_path))

if frame is None:
    print("Failed to load image")
else:
    cv2.imshow("Captured Image", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
