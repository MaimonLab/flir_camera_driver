


import PySpin
from sensor_msgs.msg import Image
from time import sleep
import cv2
from cv_bridge import CvBridge
import rclpy
from maimon_classes.basic_node import BasicNode
from collections import deque
from abc import ABC
import sys
import datetime



class CameraError(Exception):
    pass

BOOLEAN_STRING_DICT = {False: "Off", True: "On"}


class flir_container(object):

    camera_attributes={}
    camera_methods={}
    camera_node_types={}
    camera_priority_set_order = [
                                        "TriggerMode",
                                        "GainAuto",
                                        "Gain",
                                        "AcquisitionFrameRateAuto",
                                        "AcquisitionFrameRateEnabled",
                                        "AcquisitionFrameRate",
                                        "LineSelector",
                                        "LineMode",
                                        "LineSource",
                                        "LineInverter",
                                        "BinningVertical",
                                        "ExposureAuto",
                                        "Exposure",
                                    ]
    
    chunk_dict= {
                "FrameCounter": {
                    "ChunkEnable": True,
                    "ChunkModeActive": True
                },
                "Timestamp":{
                    "ChunkEnable": True,
                    "ChunkModeActive": True
                }
            }
    
    _rw_modes = {
        PySpin.RO: "read only",
        PySpin.RW: "read/write",
        PySpin.WO: "write only",
        PySpin.NA: "not available"
    }

    _attr_types = {
        PySpin.intfIFloat: PySpin.CFloatPtr,
        PySpin.intfIBoolean: PySpin.CBooleanPtr,
        PySpin.intfIInteger: PySpin.CIntegerPtr,
        PySpin.intfIEnumeration: PySpin.CEnumerationPtr,
        PySpin.intfIString: PySpin.CStringPtr,
    }

    _attr_type_names = {
        PySpin.intfIFloat: 'float',
        PySpin.intfIBoolean: 'bool',
        PySpin.intfIInteger: 'int',
        PySpin.intfIEnumeration: 'enum',
        PySpin.intfIString: 'string',
        PySpin.intfICommand: 'command',
    }
    
    def __init__(self,serial: str | None =None,id: str | None =None):
  
       
        self._system = PySpin.System.GetInstance()
        self._camlist = self._system.GetCameras()
        if serial is not None:
            self.cam=self._camlist.GetBySerial(serial)
        elif id is not None: 
            self.cam=self._camlist.GetByDeviceID(id)
        else:
            self.cam=self._camlist.GetByDeviceID(0)
            
            
        self.cam.Init()
        self.cam.DeviceReset()
        sleep(5)
        self.cam.Init()

        for node in self.cam.GetNodeMap().GetNodes():
            pit = node.GetPrincipalInterfaceType()
            name = node.GetName()
            self.camera_node_types[name] = self._attr_type_names.get(pit, pit)
            if pit == PySpin.intfICommand:
                self.camera_methods[name] = PySpin.CCommandPtr(node)
            if pit in self._attr_types:
                self.camera_attributes[name] = self._attr_types[pit](node)

        self.cam.Init()
            
    
    def start(self):
        self.running=True
        self.cam.BeginAcquisition()

    def stop(self):
        self.running=False
        self.cam.EndAcquisition()
        
    #TODO add type for timeout
    def get_img(self,timeout=PySpin.EVENT_TIMEOUT_NONE):
        
        img=self.cam.GetNextImage(timeout)
        return img.GetNDArray()
    
    def prioritize_parameters(self,parameters: dict):
        parameters_ordered=dict([(k,parameters[k]) for k in self.camera_priority_set_order if k in parameters.keys()])
        return {**parameters_ordered,**dict(parameters.items()-parameters_ordered.items())}
           
    def set_from_dict(self,parameters: dict):
        
        parameters=self.prioritize_parameters(parameters)
        self.get_logger().info(f"{parameters}")
        for attribute_name, attribute_value in parameters.items():
            try:
                self.set_cam_setting(attribute_name,attribute_value)
                #attr.SetValue(attribute_value)
            except TypeError:
                # yaml turns "On" to True, and "Off" to False under certain circumstances
                # The camera does not accept True for "On" and will ignore this setting attempt
                # if we get the type error, we can guess a remapping and retry the setting
                self.get_logger().info(
                    f"TypeError for {attribute_name}: {attribute_value}, using '{BOOLEAN_STRING_DICT[attribute_value]}'"
                )
                attribute_value = BOOLEAN_STRING_DICT[attribute_value]

                # try with converted type, otherwise throw bigger error
                try:
                    self.set_cam_setting( attribute_name, attribute_value)
                except:
                    self.get_logger().warn(
                        f"Error setting [{attribute_name}: {attribute_value}] after type conversion! skipping"
                    )
            except:
                self.get_logger().warn(
                    f"Error setting [{attribute_name}: {attribute_value}], skipping"
                )

    def get_attr(self, attr):
        if attr in self.camera_attributes:
            prop = self.camera_attributes[attr]
            if not PySpin.IsReadable(prop):
                raise CameraError("Camera property '%s' is not readable" % attr)

            if hasattr(prop, "GetValue"):
                return prop.GetValue()
            elif hasattr(prop, "ToString"):
                return prop.ToString()
            else:
                raise CameraError("Camera property '%s' is not readable" % attr)
        elif attr in self.camera_methods:
            return self.camera_methods[attr].Execute
        else:
            super().__getattribute__(self,attr)


    def set_cam_setting(self, attr, val):
        if attr in self.camera_attributes:

            prop = self.camera_attributes[attr]
            if not PySpin.IsWritable(prop):
                raise CameraError("Property '%s' is not currently writable!" % attr)
            print(attr)
            print(val)
            if hasattr(prop, 'SetValue'):
                prop.SetValue(val)
            else:
                prop.FromString(val)

        elif attr in self.camera_methods:
            raise CameraError("Camera method '%s' is a function -- you can't assign it a value!" % attr)
        
    
class flir_node(BasicNode,flir_container):
    flir=flir_container("20400194")
    print(PySpin.IsWritable(flir.camera_attributes["Gain"]))
    print(PySpin.IsWritable(flir.camera_attributes["ExposureTime"]))
    settings_key="camera_settings"
    chunk_settings_key="camera_chunkdata"
    timer_period=0.001
    
    def __init__(self,serial: str | None =None,id: str | None =None):
        BasicNode.__init__(self)
        if id is not None: raise NotImplementedError("Currently ID as well as serial is passed through serial argument")
        
        

    
       
        self.default_param = {
        "cam_id": None,
        "image_topic": "camera/image",
        "reset_camera_settings": False,
        "latch_timing_interval_s": 5,
        "add_timestamp": False,
        "flip_y": False,
        "qos_image_publish_reliable": False,
        "disregard_chunkdata": False
        }
        
        if serial is not None:
            self.cam_id=serial
        else:
            self.cam_id = self.get_parameter("cam_id").value
            #TODO this is hacky
            if (type(self.cam_id) == int) and (self.cam_id > 10):
                # If a large int, it most likely means an id, which is of a string format
                # note that leading zeros in cam id of type int would result in bugs that can only be solved before yaml is produced
                self.cam_id = f"{self.cam_id}"
            elif type(self.cam_id) == str:
                # You can prepend an id with a dollar sign in the yaml file to force the string type. We'll remove that here.
                self.cam_id = self.cam_id.replace("$", "")        

        # flag to burn timestamp in bottom left of image
        self.add_timestamp = self.get_parameter("add_timestamp").value
    
        self.bridge=CvBridge()
        self.count_published_images=0
        
        image_topic = self.get_parameter("image_topic").value
        self.pub_stream=self.create_publisher(Image,image_topic,self.get_parameter("qos_image_publish_reliable").value)
        
        
        flir_container.__init__(self,serial=self.cam_id)
        
        ros_cam_parameters = self.get_parameters_by_prefix(self.settings_key)
        
        settings_dict = dict( [(param_name, self.get_parameter(f"{self.settings_key}.{param_name}").value) 
             for param_name, _ in ros_cam_parameters.items()])
        self.get_logger().info(f"{settings_dict}")
        self.set_from_dict(settings_dict)
        self.timer = self.create_timer(self.timer_period, self.publish)
        
        chunk_params = self.get_parameters_by_prefix("camera_chunkdata")
            
        self.start()
    
        
        
    def estimate_callback_interval():
        return
        
    def publish(self):
        img=self.get_img(timeout=PySpin.EVENT_TIMEOUT_INFINITE)
        
        if self.get_parameter("flip_y").value == True:
            img = cv2.flip(img, 0)
            
        frame_id = str(self.count_published_images) 
        timestamp = self.get_clock().now().nanoseconds
        stamp = rclpy.time.Time(nanoseconds=timestamp).to_msg()
        self.count_published_images += 1 
        
        if self.add_timestamp:
            self.burn_timestamp(img, frame_id, timestamp)
        
        img_msg = self.bridge.cv2_to_imgmsg(img)
        
        img_msg.header.frame_id = frame_id 
        img_msg.header.stamp = stamp 
        self.pub_stream.publish(img_msg)
        
    
    def burn_timestamp(self, img_cv, frame_id, timestamp):
        """Burn timestamp in bottom left of the image"""

        height = len(img_cv)

        datetime_str = datetime.datetime.fromtimestamp(timestamp / 1e9).strftime(
            "%y/%m/%d %H:%M:%S"
        )

        cv2.rectangle(img_cv, (0, height - 17), (165, height), 0, -1)
        cv2.putText(
            img_cv,
            datetime_str,
            (0, height - 5),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (255, 255, 255),
            1,
        )
        cv2.rectangle(img_cv, (0, height - 34), (66, height - 17), 0, -1)
        cv2.putText(
            img_cv,
            f"id: {frame_id}",
            (0, height - 22),
            cv2.FONT_HERSHEY_PLAIN,
            1,
            (255, 255, 255),
            1,
        )


def main():
    # flir=flir_container("20400194")
    # print(PySpin.IsWritable(flir.camera_attributes["Gain"]))
    # print(PySpin.IsWritable(flir.camera_attributes["ExposureTime"]))
    rclpy.init()
    cam_node=flir_node()
    rclpy.spin(cam_node)
   


if __name__ == "__main__":
    main()