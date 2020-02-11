**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README

A collection of scripts to perform different operations with Pozyx devices.

## pozyx\_remote\_ranging

This script uses a Pozyx device to ask another one (tag) to perform a ranging operation against other set of devices (anchors). The results are returned via USB port in an stream of bytes that can be decoded using the next template:

```
[0xFA] [OxFA] [ido_low] [ido_high] [ot] [idd_low] [idd_high] [dt] [distance_low] [distance] [distance] [distance_high] [timestamp_low] [timestamp] [timestamp] [timestamp_high] [seq] [rss_low] [rss] [rss] [rss_high] [channel] [bitrate] [prf] [0xBB] [0xBB]
---
Total: 26 bytes
```

The script also support reading the IMU of the remote tag. In this case, the stream has this shape:

```
[0xFD] [OxFD] [ido_low] [ido_high] [quaternion.x] [quaternion.y] [quaternion.z] [quaternion.wight] [linear_acceleration.x] [linear_acceleration.y] [linear_acceleration.z] [angular_vel.x] [angular_vel.y] [angular_vel.z] [0xBB] [0xBB]
---
Total: 46 bytes
```
In the repository [https://github.com/valentinbarral/rosuwbranging.git](https://github.com/valentinbarral/rosuwbranging.git) can be found some ROS nodes that can perform this task and publish a new message of type ```gtec_msgs::PozyxRanging``` (this message type can be found in [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)).

The script can be configured with the next parameters (placed at the top of the script):

- **ownAnchorId**: The identifier of the device plugged on the Arduino board and connected to the computer. Example: ```ownAnchorId = 0x6757```
- **tagsIds**: A list of the Pozyx devices to be used as tags. Example: ```tagsIds[] = {0x6E5B}```
- **anchorIds**: The list of devices to be used as anchors. Example: ```anchorIds[] = {0x6752, 0x6710, 0x6732, 0x6708, 0x6047}```
- **resetOnFail**: Resets the device after N failed rangings. True/False.
- **maxResets**: Number of ranging errors accepted before reseting the reader. Integer>0.
- **readTagImu**: Reads the IMU values. True/False.
- **readTagRanging**: Performs the remote rangings. True/False.
- **logDebug**: If true, show human-readable information trough the USB port. This must be deactivate to use the ROS nodes that can be found in [https://github.com/valentinbarral/rosuwbranging.git](https://github.com/valentinbarral/rosuwbranging.git). By default is set to ```false```.


## pozyx\_local\_ranging_and_cir

This script performs a ranging operation between two Pozyx devices, and outputs the range values plus the Channel Impulse Response (CIR) samples over serial port. The message contains the next bytes:

```
[0xFA] [OxFA] [ido_low] [ido_high] [ot] [idd_low] [idd_high] [dt] [distance_low] [distance] [distance] [distance_high] [timestamp_low] [timestamp] [timestamp] [timestamp_high] [seq] [rss_low] [rss] [rss] [rss_high] [channel] [bitrate] [prf] CIR_SIZE*([sample_real_low] [sample_real_high] [sample_imaginary_low] [sample_imaginary_high]) [0xBB] [0xBB]
---
Total: 26 + CIR_SIZE*4 bytes
```

Each CIR sample is a complex number of 4 bytes. The number of samples, CIR_SIZE, depends on the *prf* value. For *prf=16* the CIR_SIZE is 996 whereas for *prf=64* the CIR_SIZE is 1016.

In the repository [https://github.com/valentinbarral/rosuwbranging.git](https://github.com/valentinbarral/rosuwbranging.git) can be found some ROS nodes that can read these packages and publish a new message of type ```gtec_msgs::PozyxRangingWithCir``` (this message type can be found in [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)).

The script can be configured with the next parameters (placed at the top of the script):

- **ownDevice**: The identifier of the device plugged on the Arduino board and connected to the computer. Example: ```ownDevice = 0x6757```
- **otherDevice**: The remote device. Example: ```otherDevice = 0x6E5B```
- **resetOnFail**: Resets the device after N failed rangings. True/False.
- **maxResets**: Number of ranging errors accepted before reseting the reader. Integer>0.
- **logDebug**: If true, show human-readable information trough the USB port. This must be deactivate to use the ROS nodes that can be found in [https://github.com/valentinbarral/rosuwbranging.git](https://github.com/valentinbarral/rosuwbranging.git). By default is set to ```false```.