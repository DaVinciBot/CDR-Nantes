import cv2
import numpy as np

# Chessboard settings — must match your printed pattern
CHESSBOARD = (9, 6)       # inner corners (not squares)
SQUARE_SIZE = 0.025       # meters — measure your printed square size!
MIN_CAPTURES = 20         # number of good frames needed

# Prepare object points
objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

obj_points = []  # 3D points
img_points = []  # 2D points

cap = cv2.VideoCapture(0)
captures = 0

print("Move the chessboard around in front of the camera.")
print("Press SPACE to capture a frame, Q to quit and calibrate.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)

    display = frame.copy()

    if found:
        # Refine corner positions
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        cv2.drawChessboardCorners(display, CHESSBOARD, corners_refined, found)
        cv2.putText(display, "Chessboard found! Press SPACE to capture",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(display, "No chessboard detected...",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.putText(display, f"Captures: {captures}/{MIN_CAPTURES}",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Calibration", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' ') and found:
        obj_points.append(objp)
        img_points.append(corners_refined)
        captures += 1
        print(f"Captured {captures}/{MIN_CAPTURES}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# --- Calibrate ---
if captures < 5:
    print("Not enough captures to calibrate. Need at least 5.")
else:
    print("Calibrating...")
    h, w = gray.shape
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None
    )
    print(f"\nCalibration error (lower is better): {ret:.4f}")
    print(f"\ncamera_matrix = \n{camera_matrix}")
    print(f"\ndist_coeffs = \n{dist_coeffs}")

    # Save to file
    np.save("camera_matrix.npy", camera_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)
    print("\nSaved to camera_matrix.npy and dist_coeffs.npy")