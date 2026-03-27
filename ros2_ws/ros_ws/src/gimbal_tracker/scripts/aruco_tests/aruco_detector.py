import cv2
import cv2.aruco as aruco

def main():
    # 1. Open the camera (0 is usually the default webcam)
    cap = cv2.VideoCapture(0)

    # 2. Define the dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 3. Detect the markers
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        # 4. If markers are found, draw them
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"Detected marker IDs: {ids.flatten()}")

        # 5. Display the result
        cv2.imshow('ArUco Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()