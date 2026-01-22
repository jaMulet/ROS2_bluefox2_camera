# ROS2 stack for BlueFOX2 cameras

## Overview

C++ packages and ROS2 wrappers to obtain images from the BlueFOX2 cameras. This repository is based on the [bluefox2](https://github.com/KumarRobotics/bluefox2) library.

This repository consists of following packages:

* *bluefox2*. Bluefox2 source files.
* *bluefox2_ros*. Main launch files.

### bluefox2

Contains main source files to communicate with bluefox2 driver API. This package requires the bluefox driver to be installed, following the [Balluff technical documentation](https://assets.balluff.com/MV_Manuals/online/mvBlueFOX/mvBF_quickstart_software.html) (prevously [Matrix Vision](https://www.balluff.com/en-de/focus-topics/balluff-mv)) for BVS CA and MLC camera models. A *.cmake* configuration file has been implemented that looks for driver libraries and includes in the default installation folder (i.e. */opt/mvIMPACT_Acquire*). Change it accordingly to your own installation.

### bluefox2_ros

All ROS2-related files are in this folder. It contains the main launcher to run the source code as a unique node. It also contains a configuration folder that stores (examples included for an MLC205G device):

* *Camera configuration file* (file name format: `camera_'device name'.config.yaml`). Defines all configuration parameters to be loaded as parameters during start-up.
* *Camera calibration file* (file name format: `calib_'device name'.yaml`). Contains a calibration file for this camera, for example, using the [ROS-perception's image_pipeline package](https://github.com/ros-perception/image_pipeline/tree/rolling).

## ROS2 API

### Topics

* `/camera/camera_info`

Outputs general camera information.

`Type: sensor_msgs/msg/CameraInfo `

* `/camera/image_raw`

Outputs images from taken from the bluefox2 camera.

`Type: sensor_msgs/msg/Image `

Complementary topics are:
/camera/image_raw/compressed
/camera/image_raw/compressedDepth
/camera/image_raw/theora

* `/diagnostics`

Outputs diagnostic messages based on the *diagnostic_updater* package.

`Type: diagnostic_msgs/msg/DiagnosticArray`

### Parameters

Included in the 'camera configuration file', and are:

1. `serial` (string). Device serial number.
2. `mode` (string). Device mode ['master' or 'slave'].
3. `frame_id` (string).
4. `mm` (integer). Sets mirror mode.
   * mm_off = 0: mirror mode off".
   * mm_topdown = 1: top down.
   * mm_leftright = 2, "Left right.
   * mm_topdown_and_leftright = 3: top down and left right.
5. `fps` (double). Frames per second [1 to 200].
6. `width` (integer). AOI width [0 to 2048].
7. `height` (integer). AOI height [0 to 2048].
8. `idpf` (integer). Defines the pixel format of the resulting image.idpf_auto = 0: the driver will decide which format to use".
   * idpf_raw = 1: an unprocessed block of data.
   * idpf_mono8 = 2: a mono channel 8 bit.
   * idpf_mono16 = 9: a mono channel 16 bit.
   * idpf_rgb888_packed = 10: RGB with 24 bit per pixel.
   * idpf_bgr888_packed = 22: BGR with 24 bit per pixel.
9. `cbm` (integer). Defines valid binning modes for the camera.
   * cbm_off = 0: no binning.
   * cbm_binning_h = 1: horizontal binning (combines 2 adjacent columns).
   * cbm_binning_v = 2: vertical binning (combines 2 adjacent rows).
   * cbm_binning_hv = 3: horizontal and vertical binning.
10. `aec` (bool). Automatic exposure control.
11. `expose_us` (integer). Exposure time for an image in microseconds [10 to 100000].
12. `agc` (bool). Automatic gain control.
13. `gain_db` (double). Gain in dB [0.0 to 32.0].
14. `acs` (integer). Defines valid AutoControlSpeed modes.
    * acs_unavailable = -1: auto control parameters not available.
    * acs_slow = 0: coverge slowly to desired value.
    * acs_medium = 1: converge to desired value at medium speed.
    * acs_fast = 2: converge fast to desired value.
15. `des_grey_value` (integer). Desired average grey value [0 to 255]
16. `hdr` (bool). High dynamic range.
17. `dcfm` (integer). Operation mode of the dark current filter.
    * dcfm_off = 0: filter is switched off.
    * dcfm_on = 1: filter is switched on.
    * dcfm_calibrate = 2: calculate dark current corrention image.
    * correction_image = 3: replace captured image with the last correction image.
18. `cpc` (integer). Defines valid camera pixel frequencies.
    * cpc_12000 = 12000: 12 Mhz.
    * cpc_20000 = 20000: 20 Mhz.
    * cpc_24000 = 24000: 24 Mhz.
    * cpc_27000 = 27000: 27 Mhz.
    * cpc_32000 = 32000: 32 Mhz.
    * cpc_40000 = 40000: 40 Mhz.
    * cpc_50000 = 50000: 50 Mhz.
19. `ctm` (integer). Camera trigger mode.
    * ctm_continuous = 0: continuously exposes images.
    * ctm_on_demand = 1: start frame expose when software asks for an image.
    * ctm_on_low_level = 2: start the exposure of a frame when the trigger input is below the trigger threshold.
    * ctm_on_high_level = 3: start the exposure of a frame when the trigger input is above the trigger threshold.
    * ctm_on_falling_edge = 4: start the exposure of a frame when the trigger input level changes from high to low.
    * ctm_on_rising_edge = 5: start the exposure of a frame when the trigger input level changes from low to high.
    * hard_sync = -1: hardware sync with master and slave (stereo only, hack).
20. `cts` (integer). Camera trigger source.
    * cts_unavailable = -1: trigger mode is continuous or on_demand.
    * cts_dig_in_0 = 0: uses digital input 0 as the source for the trigger signal.
    * cts_dig_in_1 = 1: use digital input 1 as the source for the trigger signal.
21. `request` (integer). Prefill capture queue by request [0 to 4].
22. `wbp` (integer). White balance parameter.
    * wbp_unavailable = -1: not available.
    * wbp_tungsten = 0: tungsten.
    * wbp_halogen = 1: halogen.
    * wbp_fluorescent = 2: fluorescent.
    * wbp_daylight = 3: day light.
    * wbp_photolight = 4: photo Light.
    * wbp_bluesky = 5: blue Sky.
    * wbp_user1 = 6: user1.
    * wbp_calibrate = 10: calibrate.
23. `r_gain` (integer).
24. `g_gain` (integer).
25. `b_gain` (integer).
26. `camera_calibration_file` (string). URL containing path to the camera calibration file. File name format: `calib_'device name'.yaml`.

# Usage

## Configuration

Firstly, the *camera_configuration_file* must be populated with the parameters according to the hardware that will be connected to.

## Launch

Launch the node that publishes camera images on /camera/image_raw.

```
ros2 launch bluefox2_ros single_node.launch.py device:='' namespace:='' calib_url:=''
```

Tentative arguments are:

* *device* (or 'device name'). Only used when looking for configuration and calibration file by name. This is, files' name must match whit the name given here. E.g.: `MLC205G`.
* *namespace*. Gives a namespace to the node when running multiple cameras.
* *calib_url*. URL containing path to the camera calibration file, that will be loaded as parameter to be used by the node. By default, the launch file looks for the file `calib_'device name'.yaml` in the `config` folder once compiled. E.g. `file://.../install/bluefox2_ros/share/bluefox2_ros/config/calib_MLC205G.yaml`
