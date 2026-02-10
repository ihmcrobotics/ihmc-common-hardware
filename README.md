# ihmc-common-hardware

Project containing drivers/classes for boards, sensors, motor controllers, mechanisms, and other hardware commonly used on IHMC robots.

## ihmc-common-hardware-devices

This subproject contains code for etherCAT device communication and control, as well as abstract classes for creating a hardware device map and manager

### Structure

- commonHardware: Main directory, contains abstract classes for creating a hardware map and manager
    - devices: Contains code for commonly used devices (IMU, encoder, load cell) and interfaces for easy integration of new devices into the system
    - mechanisms: Contains code for managing actuation mechanisms and interfaces for integration of new types into the system. Currently, only cycloids have a
      full implementation

## ihmc-hardware-status-ui

This subproject contains code for monitoring and displaying the current status of the devices on the robot in a tabular form.

### Structure
- hardwareStatusUI: Main directory
    - controllerSide: Holding the current status of a device and providing it for the visualizer
    - visualizerSide: Creating the table visualization and implementing it within SCS2

## ihmc-hardware-xml-toolkit

This subproject contains code to translate xml-formatted robot descriptions into device/mechanism settings and parameters

### Structure
- hardwareXmlToolkit: Main directory, contains classes for a general hardware description and the loader used to translate descriptions
    - devices: Translating device descriptions from xml formatting
    - joints: Translating joint descriptions from xml formatting
    - priority: Translating thread priority settings from xml formatting
    - settings: Translating communication (EtherCAT, ROS2) settings from xml formatting
    - transmissions: Translating transmission settings from xml formatting


## How to Implement for your own robot

There are four steps to follow for basic implementation for your own robot using devices already implemented:

1. Format the xml files describing the robot to match [
   `XmlHardwareDescription`](https://github.com/ihmcrobotics/ihmc-common-hardware/tree/develop/ihmc-hardware-xml-toolkit/src/main/java/us/ihmc/hardwareXMLToolkit/XmlHardwareDescription.java),
   and make sure the description of each device and mechanism is formatted so that
   they can be properly loaded by [
   `XmlHardwareDescriptionLoader`](https://github.com/ihmcrobotics/ihmc-common-hardware/tree/develop/ihmc-hardware-xml-toolkit/src/main/java/us/ihmc/hardwareXMLToolkit/XmlHardwareDescriptionLoader.java).
   An example can be found [here](https://github.com/ihmcrobotics/ihmc-common-hardware/tree/develop/ihmc-hardware-xml-toolkit/src/main/resources/xmlExamples)
2. Create a class that extends [
   `AbstractHardwareMap`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-common-hardware-devices/src/main/java/us/ihmc/commonHardware/AbstractHardwareMap.java)
   to properly create and register all devices and mechanisms
3. Create a class that extends [`AbstractHardwareManager`](https://github.com/ihmcrobotics/ihmc-common-hardware/blob/develop/ihmc-common-hardware-devices/src/main/java/us/ihmc/commonHardware/AbstractHardwareManager.java) to properly manage reading and writing to each device and mechanism
4. Create a controller that can communicate using EtherCAT. You can either extend the pre-made [
   `EtherCATRealtimeThread`](https://github.com/ihmcrobotics/ihmc-ethercat-master/blob/develop/src/main/java/us/ihmc/etherCAT/master/EtherCATRealtimeThread.java)
   for simplicity or implement your own
   EtherCAT control by implementing [
   `MasterInterface`](https://github.com/ihmcrobotics/ihmc-ethercat-master/blob/develop/src/main/java/us/ihmc/etherCAT/master/MasterInterface.java). Both
   classes are part of [ihmc-ethercat-master](https://github.com/ihmcrobotics/ihmc-ethercat-master).

## Making a Release
- Bump the version in `group.product.properties`
- Follow the remaining steps [here](https://ihmcrobotics.atlassian.net/wiki/spaces/HOWTO/pages/1497530369/Default+Release+a+Library) (skip the version bump step in that guide, since you have already done it)

## Maintainers
* Reese Peterson (rpeterson@ihmc.org)
* Stefan Fasano (sfasano@ihmc.org)
* Dexton Anderson (danderson@ihmc.org)
* Duncan Calvert (dcalvert@ihmc.org)
* Robert Griffin (rgriffin@ihmc.org)
