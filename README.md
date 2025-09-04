# IHMC Common Hardware

Project containing drivers/classes for boards, sensors, motor controllers, mechanisms, and other hardware commonly used on IHMC robots.

## Hardware Devices

This package holds classes for device communication and control, as well as abstract classes for creating a hardware device map and manager. 
At current, communication is done via EtherCAT for all devices. 

### Structure

- commonHardware: Main directory. Holds Abstract classes for creating a hardware map and manager
    - devices: Contains code for commonly used devices (IMU, encoder, load cell) and interfaces for easy integration of new devices into the system
    - mechanisms: Contains code for managing actuation mechanisms and interfaces for integration of new types into the system. Currently, only cycloids have a
      full implementation

## Hardware Status UI

This package contains classes to handle gathering and displaying the device statuses on the robot

### Structure

- hardwareStatusUI: Main directory
  - controllerSide: Contains interfaces for device classes to extend to provide statuses and classes for holding and managing statuses
  - visualizerSide: Contains classes to intake device statuses and display them. Includes abstract classes to implement the UI into a visualizer


## Hardware XML Toolkit

This package contains code for taking XML descriptions of robot hardware for all devices and mechanisms in the project into parameters used for creating them.

### Structure

- hardwareXMLToolkit: Main directory. Holds abstract class for any XML object and the general XML description with the respective description loader
  - devices: Contains classes for the XML description of each device and the XML description and loader for the cycloid parameters
  - joints: Contains classes for XML descriptions of joints in the robot. Only needed if the joint is passive and sensed, can be used to tie joint to encoder
  - priority: Contains classes for setting threading affinities and priorities for multi-threading
  - settings: Contains classes for EtherCAT and ROS settings for the robot
  - transmissions: Contains classes for the XML description of common transmission systems

### How to Implement for your own robot

There are four steps to follow for basic implementation for your own robot using devices already implemented:

1. Format the xml files describing the robot to match [`XmlHardwareDescription`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-hardware-xml-toolkit/src/main/java/us/ihmc/hardwareXMLToolkit/XmlHardwareDescription.java), and make sure the description of each device and mechanism is formatted so that they can be properly loaded by [`XmlHardwareDescriptionLoader`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-hardware-xml-toolkit/src/main/java/us/ihmc/hardwareXMLToolkit/XmlHardwareDescriptionLoader.java). An example can be found [here](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-hardware-xml-toolkit/src/main/resources/xmlExamples/Robot.xml).
2. Create a class that extends [`AbstractHardwareMap`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-common-hardware-devices/src/main/java/us/ihmc/commonHardware/AbstractHardwareMap.java) to properly create and register all devices and mechanisms.
3. Create a class that extends [`AbstractHardwareManager`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-common-hardware-devices/src/main/java/us/ihmc/commonHardware/AbstractHardwareManager.java) to properly manage reading and writing to each device and mechanism.
4. Create a controller that can communicate using EtherCAT. You can either extend the pre-made [`EtherCATRealtimeThread`](https://github.com/ihmcrobotics/ihmc-ethercat-master/blob/develop/src/main/java/us/ihmc/etherCAT/master/EtherCATRealtimeThread.java) for simplicity, or implement your own EtherCAT control by implementing [`MasterInterface`](https://github.com/ihmcrobotics/ihmc-ethercat-master/blob/develop/src/main/java/us/ihmc/etherCAT/master/MasterInterface.java). Both classes are part of [ihmc-ethercat-master](https://github.com/ihmcrobotics/ihmc-ethercat-master).

To create an SCS vizualizer to view hardware statuses using the UI, there are two main steps if using devices already implemented:

1. Create a class that extends ['AbstractUIHardwareStatusManager'](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-hardware-status-ui/src/main/java/us/ihmc/hardwareStatusUI/visualizerSide/AbstractUIHardwareStatusManager.java) to properly manage the statuses being passed from the devices
2. Create a visualizer that extends ['AbstractHardwareSCS2Vizualizer'](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-hardware-status-ui/src/main/java/us/ihmc/hardwareStatusUI/visualizerSide/AbstractHardwareSCS2Visualizer.java) to include the hardware status UI as an option to view.

## Maintainers

* Reese Peterson (rpeterson@ihmc.org)
* Stefan Fasano (sfasano@ihmc.org)
* Dexton Anderson (danderson@ihmc.org)
* Duncan Calvert (dcalvert@ihmc.org)
* Robert Griffin (rgriffin@ihmc.org)
