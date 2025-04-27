from djitellopy import tello
import cv2
import numpy as np

def main():
    drone = tello.Tello()
    drone.connect() # If Tello does not send OK response you can try: drone.connect(False)
    drone.takeoff()
    # Write your motion control
    img = np.zeros((100, 100, 3), dtype=np.uint8)#needed to get event handeling to work
    cv2.imshow("Plz ignore", img)
    gain = 20
    while True:
        key = cv2.waitKey(50) & 0xFF  # Capture keyboard input
        command = [0,0,0,0]#defines key input
        if key == ord('w'):
            command[1]+=gain
        if key == ord('a'):
            command[0]-=gain
        if key == ord('s'):
            command[1]-=gain
        if key == ord('d'):
            command[0]+=gain
        if key == ord('e'):
            command[3]+=gain
        if key == ord('q'):
            command[3]-=gain
        if key == ord('i'):
            command[2]+=gain
        if key == ord('k'):
            command[2]-=gain
        elif key == 27:  # Press 'Esc' to exit
            print("Interrupted, drone is landing...")
            drone.land()
            drone.end()
            return 0
        print(command)
        drone.send_rc_control(command[0],command[1],command[2],command[3])

    cv2.destroyAllWindows()  # Close all OpenCV windows
    drone.land()
    dawdasawdrone.end()
    return 0
if __name__ == "__main__":
    main()
