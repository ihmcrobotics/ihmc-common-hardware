package us.ihmc.hardwareStatusUI.controllerSide;

import us.ihmc.etherCAT.master.Slave;

/**
 * Interface for any class that wishes to provide status info of a CAN device.
 * If registered with {@code HardwareStatusManager}, this information can be displayed
 * in the {@code HardwareStatusUI}.
 */
public interface EtherCATDeviceStatusProvider extends DeviceStatusProvider
{
   Slave.State getState();
}
