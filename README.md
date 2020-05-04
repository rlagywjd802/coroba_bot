# corona_bot
These are packages for disinfection robot ADAMMS_UV.

## Hardware Components
### actuator
- inspectorbot
- ur5
- robotiq gripper

### sensor
- kinect
- realsense
- webcam (x3)

## Installation
Before running the code, make sure that you have installed following packages.
- openni_launch(kinect)
- realsense-ros(realsense)
- usb_cam(webcam)
- ur_modern_driver(ur5)
- universal_robot(ur5)
- robotiq(gripper)
- PCLFusionColor
- ApproximateBoundingBox
- mobile_disinfectant_rob

## Usage
On robot computer, run:
```roslaunch cb_bringup cb_bringup.launch rviz:=false cam1_device:=<video_device_for_cam1> cam2_device:=<video_device_for_cam1> cam3_device:=<video_device_for_cam1>```
```roslaunch cb_bringup cb_plan_for_clicked_point.launch```

Before running the command, you should check the video device for three webcams.
To list the video device, run:
```ls -la /dev/video*```
And change the access permision for each of devices as follows.
```sudo chmod 666 <video_device>```

You can either directly launch above files from robot computer or launch via ssh on remote computer.
To access robot computer via ssh, run:
```ssh -X mm@mm```

On remote computer, run:
```roslaunch cb_bringup cb_bringup_rviz.launch```