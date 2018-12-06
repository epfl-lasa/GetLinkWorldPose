## GetLinkWorldPose
Gazebo Plugin to get any 6D frame of a robot link and write it to a yarp-port, used with iCub.

```c++
// ====== Opening the port for Root-Link in World (CoM) Pose reader w/robotName ====== //
std::string RootlinkPose_portName="/";
RootlinkPose_portName += robotName_;
RootlinkPose_portName += "/get_root_link_WorldPose:o";


RootlinkPose_port_In.open("/RootlinkPose_In:i");
if(!Network::connect(RootlinkPose_portName.c_str(), RootlinkPose_port_In.getName().c_str())){
    printf(" Unable to connect to the KeyboardCmdsReaderModule port");
    return false;
}
```
