from djitellopy import tello
import cv2
import time
def main():
    drone = tello.Tello(host='192.168.10.101')
    drone.connect() # If Tello does not send OK response you can try: drone.connect(False)
    drone.takeoff()
    # Write your motion control
    drone.move("back", 150)
    drone.move("left", 150)
    drone.move("forward", 150)
    drone.move("right", 150)
    drone.land()
    drone.end()

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        print("Interrupted, drone is landing...")
        drone.land()
        drone.end()
        return 0

    return 0
if __name__ == "__main__":
    main()
