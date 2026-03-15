# aruco_detector.py

import cv2
from cv2 import aruco
import numpy as np
import time
import threading

class ArucoDetector:
    """
    Detects ArUco markers and returns object info.
    
    Usage:
        detector = ArucoDetector(camera_index=0)
        detector.start()
        
        info = detector.get_detections()
        # info is a list of dicts:
        # [{ 'id': 0, 'label': 'Block #1', 'distance': 0.45,
        #    'angle_to_face': -12.3, 'yaw': 5.1, 'pitch': 0.2, 'roll': 1.0 }]
        
        detector.stop()
    """

    OBJECT_MAP = {
        0:  "Block #1",
        29: "Block #2",
    }

    MARKER_SIZE = 0.05  # meters

    def __init__(self, camera_index=0, target_fps=15,
                 camera_matrix=None, dist_coeffs=None):

        self.camera_index = camera_index
        self.target_fps = target_fps
        self.frame_interval = 1.0 / target_fps

        # Load calibration or fall back to defaults
        if camera_matrix is not None:
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
        else:
            try:
                self.camera_matrix = np.load("camera_matrix.npy")
                self.dist_coeffs = np.load("dist_coeffs.npy")
                print("[ArucoDetector] Loaded calibration from files.")
            except FileNotFoundError:
                print("[ArucoDetector] WARNING: No calibration found, using defaults.")
                self.camera_matrix = np.array([[800, 0, 320],
                                               [0, 800, 240],
                                               [0,   0,   1]], dtype=float)
                self.dist_coeffs = np.zeros((5, 1))

        self.obj_points = np.array([
            [-self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
            [ self.MARKER_SIZE/2,  self.MARKER_SIZE/2, 0],
            [ self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0],
            [-self.MARKER_SIZE/2, -self.MARKER_SIZE/2, 0]
        ], dtype=np.float32)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Thread-safe detection results
        self._detections = []
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        """Start the detection thread."""
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[ArucoDetector] Started.")

    def stop(self):
        """Stop the detection thread."""
        self._running = False
        if self._thread:
            self._thread.join()
        print("[ArucoDetector] Stopped.")

    def get_detections(self):
        """
        Returns latest detections as a list of dicts.
        Thread-safe — call this from anywhere.
        """
        with self._lock:
            return list(self._detections)

    def _run(self):
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        last_time = time.time()

        while self._running:
            ret, frame = cap.read()
            if not ret:
                continue

            # Framerate limiter
            now = time.time()
            if now - last_time < self.frame_interval:
                continue
            last_time = now

            detections = self._process_frame(frame)

            with self._lock:
                self._detections = detections

        cap.release()

    def _process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []

        if ids is None:
            return results

        for i, marker_id in enumerate(ids.flatten()):
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corners[i],
                self.camera_matrix, self.dist_coeffs
            )
            if not success:
                continue

            tvec = tvec.flatten()
            rvec = rvec.flatten()

            distance = float(np.linalg.norm(tvec))
            angle_to_face = float(np.degrees(np.arctan2(tvec[0], tvec[2])))

            rot_matrix, _ = cv2.Rodrigues(rvec)
            euler_angles = np.degrees(np.array([
                np.arctan2(rot_matrix[2][1], rot_matrix[2][2]),
                np.arctan2(-rot_matrix[2][0], np.sqrt(rot_matrix[2][1]**2 + rot_matrix[2][2]**2)),
                np.arctan2(rot_matrix[1][0], rot_matrix[0][0])
            ]))

            results.append({
                'id':            int(marker_id),
                'label':         self.OBJECT_MAP.get(int(marker_id), f"ID {marker_id}"),
                'distance':      round(distance, 4),
                'angle_to_face': round(angle_to_face, 2),
                'roll':          round(float(euler_angles[0]), 2),
                'pitch':         round(float(euler_angles[1]), 2),
                'yaw':           round(float(euler_angles[2]), 2),
            })

        return results


# --- Standalone test (only runs when you execute this file directly) ---
if __name__ == "__main__":
    detector = ArucoDetector(camera_index=0, target_fps=15)
    detector.start()

    cap = cv2.VideoCapture(0)
    aruco_draw_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.get_detections()
        for d in detections:
            print(d)  # or overlay on frame if you want visual

        cv2.imshow("ArucoDetector Test", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    detector.stop()