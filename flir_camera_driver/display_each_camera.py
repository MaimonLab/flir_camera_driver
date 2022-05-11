from simple_pyspin import Camera, list_cameras
import cv2
import time


def main():
    """Displays each camera as a running video window with the serial number.

    Iterates through each camera and displays it on screen along with its SN,
    allowing SNs to be recorded.  Type N for next camera or Q to quit.

    fixme: currently hangs / freezes after some amount of time / frames. during
        the cam.get_array() call, unclear why."""
    num_cams = len(list_cameras())
    for cam_num in range(num_cams):

        with Camera(cam_num) as cam:
            cam.init()
            setattr(cam, "AcquisitionFrameRateAuto", "Off")

            setattr(cam, "AcquisitionFrameRateEnabled", True)
            setattr(cam, "AcquisitionFrameRate", 5)

            print(f"starting camera {cam.DeviceID}")
            print(f"starting camera {cam.DeviceSerialNumber}")
            cam.start()

            while True:
                pre_time = time.time()
                # print("pre_get")

                img_cv = cam.get_array()
                print(f"aq_t: {time.time() - pre_time}")
                # print("post_get")
                height = len(img_cv)
                cv2.putText(
                    img_cv,
                    f"Press N for next Camera, Q to quit.",
                    (0, height // 5),
                    # (0, height - 5),
                    cv2.FONT_HERSHEY_COMPLEX,
                    2,
                    (255, 255, 255),
                    3,
                )
                cv2.putText(
                    img_cv,
                    f"Camera: {cam.DeviceID}",
                    (0, height // 2),
                    # (0, height - 5),
                    cv2.FONT_HERSHEY_COMPLEX,
                    3,
                    (255, 255, 255),
                    3,
                )
                # print("pre_show")
                cv2.imshow("image", img_cv)
                # print("show")
                key = cv2.waitKey(1)
                # print("wait")
                if key & 0xFF == ord("n"):
                    break
                if key & 0xFF == ord("q"):
                    return

        # break


if __name__ == "__main__":
    main()
