import cv2
import PySpin
from flir_camera_driver.ros_pyspin import Camera


def main():
    """Displays each camera as a running video window with the serial number.

    Iterates through each camera and displays it on screen along with its SN,
    allowing SNs to be recorded.  Type N for next camera or Q to quit.

    fixme: currently hangs / freezes after some amount of time / frames. during
        the cam.get_array() call, unclear why."""
    _system = PySpin.System.GetInstance()

    cameras = _system.GetCameras()
    if not cameras.GetSize():
        raise RuntimeError("No cameras found!")
    for idx, cam in enumerate(cameras):
        roscam = Camera(cam_id=idx)
        print(f"Camera {idx}: {roscam.get_attr('DeviceSerialNumber')}")
        roscam.open_cam()
        roscam.start()
        while True:
            # immutable
            img_cv, _ = roscam.get_new_frame(get_chunk=False)
            height = len(img_cv) 
            # make a mutable version of img_cv
            img_cv = img_cv.copy()
            cv2.putText(
                img_cv,
                f"Press N for next Camera, Q to quit.",
                (0, height // 5),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                (255, 255, 255),
                1,
            )

            cv2.putText(
                img_cv,
                f"Camera: {roscam.get_attr('DeviceSerialNumber')}",
                (0, height // 2),
                cv2.FONT_HERSHEY_COMPLEX,
                1.8,
                (255, 255, 255),
                2,
            )

            cv2.imshow("image", img_cv)
            key = cv2.waitKey(1)
            if key & 0xFF == ord("n"):
                break
            if key & 0xFF == ord("q"):
                return
    print("Done!")
    # break


if __name__ == "__main__":
    main()
