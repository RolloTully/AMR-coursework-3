from djitellopy import tello
import cv2

def main():
    drone = tello.Tello()
    drone.connect() # If Tello does not send OK response you can try: drone.connect(False)
    drone.takeoff()
    # Write your motion control



    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        print("Interrupted, drone is landing...")
        drone.land()
        drone.end()
        return 0
    drone.land()
    drone.end()
    return 0
if __name__ == "__main__":
    main()
