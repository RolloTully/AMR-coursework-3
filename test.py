import cv2
import numpy as np

def main():
    # Create a blank image to display (necessary for cv2.waitKey to work)
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.imshow("Press 'a' to test", img)

    while True:
        key = cv2.waitKey(1) & 0xFF  # Capture keyboard input
        if key == ord('w'):
            print("Pressed 'a'")
        if key == ord('a'):
            print("Pressed 'a'")
        if key == ord('s'):
            print("Pressed 'a'")
        if key == ord('d'):
            print("Pressed 'a'")
        if key == ord('e'):
            print("Pressed 'a'")
        if key == ord('q'):
            print("Pressed 'a'")
        elif key == 27:  # Press 'Esc' to exit
            break

    cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == "__main__":
    main()
