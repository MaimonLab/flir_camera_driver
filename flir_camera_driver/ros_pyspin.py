import cv2
import numpy as np
import PySpin
from time import sleep
import traceback
import warnings


class Camera:
    def __init__(self, cam_id=0):
        self._attr_types = {
            PySpin.intfIFloat: PySpin.CFloatPtr,
            PySpin.intfIBoolean: PySpin.CBooleanPtr,
            PySpin.intfIInteger: PySpin.CIntegerPtr,
            PySpin.intfIEnumeration: PySpin.CEnumerationPtr,
            PySpin.intfIString: PySpin.CStringPtr,
        }
        self._cam_attr = {}
        self._cam_methods = {}

        self._system = PySpin.System.GetInstance()
        self._cam_list = self._system.GetCameras()
        self._cam_id = cam_id if cam_id else 0

        self.cam = None
        self._is_running = False
        self._img_cache = None
        self.cam = self.open_cam()
        self.size = (int(self.get_attr('Width')), int(self.get_attr('Height')))

    def get_new_frame(self, get_chunk=False):
        try:
            self._img_cache = self.cam.GetNextImage(5000)
            if get_chunk:
                return self._img_cache.GetNDArray(), self._img_cache.GetChunkData()
            else:
                return self._img_cache.GetNDArray(), None

        except Exception as e:
            warnings.warn(
                f'Timeout on GetNextImage from Camera with serial: {self._cam_id}!'
                f'\nResetting camera and trying again...'
            )
            self.reset_settings()
            return self.get_new_frame(get_chunk)

    def get_timestamp(self):
        self._cam_methods['TimestampLatch'].Execute()
        return self.get_attr('Timestamp')

    def set_cam_settings(self, settings: dict):
        for k, v in settings.items():
            try:
                self.set_attr(k, v)
            except RuntimeWarning as e:
                print(e)
                continue

    def set_chunk_settings(self, settings: dict):
        for prop, val in settings.items():
            try:
                _entry = self._cam_attr['ChunkSelector'].GetEntryByName(prop)
                if _entry is not None:
                    self.set_attr('ChunkSelector', _entry.GetValue())
                else:
                    print(f'Could not find ChunkSelector for {prop}! Skipping...')
                    continue
            except RuntimeWarning:
                continue
            for switch, v in val.items():
                try:
                    self.set_attr(switch, v)
                except RuntimeWarning as e:
                    print(e)
                    continue

    def get_attr(self, attr):
        try:
            if attr not in self._cam_attr:
                return
            if PySpin.IsReadable(self._cam_attr[attr]):
                if hasattr(self._cam_attr[attr], 'GetValue'):
                    return self._cam_attr[attr].GetValue()
                elif hasattr(self._cam_attr[attr], 'GetCurrentEntry'):
                    # attributes of type PySpin.CEnumerationPtr comprise
                    # a list of entries or "modes" that the attribute that be in,
                    # so we need to fetch the current entry instead of a value
                    return self._cam_attr[attr].GetCurrentEntry().GetName()
                elif hasattr(self._cam_attr[attr], 'GetIntValue'):
                    return self._cam_attr[attr].GetIntValue()
            return
        except:
            return

    def set_attr(self, attr, val):
        try:
            if attr in self._cam_attr:
                prop = self._cam_attr[attr]
                if not PySpin.IsWritable(prop):
                    raise PermissionError

                if hasattr(prop, 'SetValue'):
                    prop.SetValue(val)
                elif hasattr(prop, 'SetIntValue') and isinstance(val, int):
                    prop.SetIntValue(val)
                elif hasattr(prop, 'GetEntryByName'):
                    # attributes of type PySpin.CEnumerationPtr requires an integer index
                    # corresponding to a "mode" that the attribute can be switched to
                    prop.SetIntValue(prop.GetEntryByName(val).GetValue())
                else:
                    prop.FromString(val)
                print(f'Succesfully set {attr} to {val}!')

            else:
                setattr(self.cam, attr, val)
                print(f'Succesfully set cam.{attr} to {val}!')

        except TypeError:
            if isinstance(val, bool):
                self.set_attr(attr, 'On' if val else 'Off')
            else:
                raise RuntimeWarning(f'Error settings attribute {attr}! Skipping...')

        except Exception as e:
            print(e)
            raise RuntimeWarning(f'Error settings attribute {attr}! Skipping...')

    def reset_settings(self):
        self._cam_id = self.get_attr('DeviceSerialNumber')
        self._cam_methods['DeviceReset'].Execute()
        self.cam.DeInit()
        self.cam = None
        while self.cam is None:
            sleep(5)
            try:
                self._cam_list = self._system.GetCameras()
                self.cam = self.open_cam()
            except RuntimeError:
                continue

    def open_cam(self):
        if not self._cam_list.GetSize():
            raise RuntimeError('No cameras detected!')

        try:
            if isinstance(self._cam_id, int) and (self._cam_id > 10):
                # If a large int, it most likely means an id, which is of a string format
                # note that leading zeros in cam id of type int
                # would result in bugs that can only be solved before yaml is produced
                self._cam_id = f"{self._cam_id}"
            if isinstance(self._cam_id, str):
                # You can prepend an id with a dollar sign in the yaml file
                # to force the string type. We'll remove that here.
                self._cam_id = self._cam_id.replace("$", "")
                self.cam = self._cam_list.GetBySerial(self._cam_id)
            else:
                self.cam = self._cam_list.GetByIndex(self._cam_id)

            self.cam.Init()

            for node in self.cam.GetNodeMap().GetNodes():
                pit = node.GetPrincipalInterfaceType()
                name = node.GetName()
                if pit == PySpin.intfICommand:
                    self._cam_methods[name] = PySpin.CCommandPtr(node)
                elif pit in self._attr_types:
                    self._cam_attr[name] = self._attr_types[pit](node)

            return self.cam

        except BaseException:
            raise RuntimeError(
                f"Failed to open Camera with serial: {self._cam_id}! "
                f"It might be opened elsewhere or not plugged in."
            )

    def start(self):
        self.cam.BeginAcquisition()
        self._is_running = True

    def destroy(self):
        self._is_running = False
        self._img_cache.Release()
        self.cam.EndAcquisition()
        self.cam.DeInit()
        del self.cam
        self._cam_list.Clear()
        self._system.ReleaseInstance()


if __name__ == '__main__':
    cam = Camera(0)
    cam.reset_settings()
    cam.set_cam_settings({
        'TriggerMode': 'Off',
        'AcquisitionFrameRateAuto': 'Off',
        'AcquisitionFrameRateEnabled': True,
        'AcquisitionFrameRate': 60,
    })
    cam.set_chunk_settings({
        'FrameCounter': {'ChunkEnable': True, 'ChunkModeActive': True},
        'Timestamp': {'ChunkEnable': True, 'ChunkModeActive': True}
    })

    cam.start()
    cv2.namedWindow('Preview')
    while True:
        key = cv2.waitKey(1)
        if key == 32:
            break
        f, chunk = cam.get_new_frame(get_chunk=True)
        cv2.imshow('Preview', f)
    cam.destroy()
