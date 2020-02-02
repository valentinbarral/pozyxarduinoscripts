**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README

A collection of scripts to perform different operations with Pozyx UWB nodes.

## pozyx\_remote\_ranging.ino

This script uses a Pozyx device to ask another one (tag) to perform a ranging operation against other set of devices (anchors). The results are returned via USB port in an stream of bytes that can be decoded using the next template:

```
[0xFA] [OxFA] [ido_low] [ido_high] [ot] [idd_low] [idd_high] [dt] [distance_low] [distance] [distance] [distance_high] [timestamp_low] [timestamp] [timestamp] [timestamp_high] [seq] [rss_low] [rss] [rss] [rss_high] [channel] [bitrate] [prf] [0xBB] [0xBB]
---
Total: 26 bytes
```
Additionally, in the repository [https://github.com/valentinbarral/rosuwbranging.git](https://github.com/valentinbarral/rosuwbranging.git) can be found some ROS nodes that can perform this task and publish a new message of type ```gtec_msgs::PozyxRanging``` (this message type can be found in [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)).

The script can be configured with the next parameters (placed at the top of the script):

- **ownAnchorId**: The identifier of the device plugged on the Arduino board and connected to the computer. Example: ```ownAnchorId = 0x6757```
- **tagsIds**: A list of the Pozyx devices to be used as tags. Example: ```tagsIds[] = {0x6E5B}```
- **anchorIds**: The list of devices to be used as anchors. Example: ```anchorIds[] = {0x6752, 0x6710, 0x6732, 0x6708, 0x6047}```
- **maxResets**: Number of ranging errors accepted before reseting the reader.
- **logDebug**: If true, show human-readable information trough the USB port. This must be deactivate to use the ROS nodes that can be found in [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs). By default is set to ```false```.


