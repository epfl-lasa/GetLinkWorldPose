## GetLinkWorldPose
Gazebo Plugin to get any 6D frame of a robot link and write it to a yarp-port, used with iCub.

### Compilation and build
Clone the repository
```bash
$ git clone https://github.com/epfl-lasa/GetLinkWorldPose.git
```

To define the link you want to read from Gazebo, define in the ``GetWorldLinkPose.cc`` file:
```c++
    // get the link name
    std::string linkName = "";

    if(this->m_defaultLink =="")
    {
        linkName = rf.find("link").asString();
        if(linkName == ""){
            linkName = "root_link";
        }
    }
    else{
        linkName = rf.find("link").asString();
        if(linkName == ""){
            linkName = this->m_defaultLink;
        }
    }
```
Then compile:
```bash
$ cd ~/GetLinkWorldPose
$ mkdir build && cd build
$ cmake .. && make
```
This will create a the shared object ``libPlugin_LinkWorldPose.so`` in the ``./build`` folder.

### Usage
To actually use the plugin, you should dd this plugin in icub sdf model for instance the one in icub-gazebo folder as follows:
```xml
<plugin name='get_link_world_pose' filename='/where/you/have/your/libPlugin_LinkWorldPose.so'>
  <robotNamefromConfigFile>model://icub/conf/gazebo_icub_robotname.ini</robotNamefromConfigFile>>
</plugin>
```

Then in your C++ code, you can connect to the yarp port, as follows:
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

    RootlinkPose_values = RootlinkPose_port_In.read(); 
    Rootlink_measurements.resize(RootlinkPose_values->size());    
    for (int i= 0;i < RootlinkPose_values->size(); i++){
        Rootlink_measurements(i) = RootlinkPose_values->get(i).asDouble();            
    }
```
