import cv2
import numpy as np

# ========= CONFIG =========
CAMERA_INDEX = 2

# Crop rectangle: top-left corner (X, Y), width, height
CROP_X = 550
CROP_Y = 170
CROP_W = 300
CROP_H = 250

# True = grayscale intensity, False = raw channel average
USE_GRAYSCALE_INTENSITY = True
# ==========================


def compute_avg_intensity(frame, x, y, w, h, use_grayscale=True):
    frame_h, frame_w = frame.shape[:2]

    x = max(0, x)
    y = max(0, y)
    w = max(1, w)
    h = max(1, h)

    x2 = min(frame_w, x + w)
    y2 = min(frame_h, y + h)

    if x >= x2 or y >= y2:
        raise ValueError("Crop region is outside the frame bounds.")

    crop = frame[y:y2, x:x2]

    if use_grayscale:
        gray_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        avg_intensity = float(np.mean(gray_crop))
    else:
        avg_intensity = float(np.mean(crop))

    return avg_intensity, crop, (x, y, x2, y2)


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open webcam at index {CAMERA_INDEX}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from webcam")
            break

        try:
            avg_intensity, crop, (x1, y1, x2, y2) = compute_avg_intensity(
                frame,
                CROP_X,
                CROP_Y,
                CROP_W,
                CROP_H,
                use_grayscale=USE_GRAYSCALE_INTENSITY,
            )

            display = frame.copy()

            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                display,
                f"Avg intensity: {avg_intensity:.2f}",
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            print(f"\rAverage intensity: {avg_intensity:.2f}", end="")

            cv2.imshow("Webcam with Crop Overlay", display)
            cv2.imshow("Cropped Region", crop)

        except ValueError as e:
            print(f"\n{e}")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    print()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()