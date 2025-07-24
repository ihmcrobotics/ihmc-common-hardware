package us.ihmc.commonHardware.hardwareStatusUI.controllerSide;

import us.ihmc.commonHardware.xmlDescription.devices.AbstractXmlDevice;
import us.ihmc.commonHardware.xmlDescription.devices.AbstractXmlEtherCATDevice;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;

public class HardwareStatusManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ArrayList<DeviceStatusHolder> deviceStatusHolders = new ArrayList<>();

   /**
    * Manages and updates all {@code DeviceStatusHolder} and {@code DeviceStatusProvider}.
    * This class is ultimately responsible for handling the exchange of hardware status
    * information from each device's java class, to the {@code HardwareStatusUI} where
    * that status data is visualized.
    */
   public HardwareStatusManager(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   /**
    * Registers a generic device and status provider by creating a {@code DeviceStatusHolder} and adding it to the list
    *
    * @param xmlDevice            Generic device description taken from an xml
    * @param deviceStatusProvider Status provider for the specific device
    */
   public void registerDevice(AbstractXmlDevice xmlDevice, DeviceStatusProvider deviceStatusProvider)
   {
      deviceStatusHolders.add(new DeviceStatusHolder(xmlDevice.getName(), deviceStatusProvider, registry));
   }

   /**
    * Registers a generic EtherSnacks device and status provider by creating a {@code DeviceStatusHolder} and adding it to the list
    *
    * @param xmlDevice            EtherSnacks device to be added
    * @param xmlParentDevice      Parent EtherSnacks board
    * @param deviceStatusProvider Status provider for the EtherSnacks device
    */
   public void registerDevice(AbstractXmlDevice xmlDevice, AbstractXmlDevice xmlParentDevice, EtherCATDeviceStatusProvider deviceStatusProvider)
   {
      deviceStatusHolders.add(new DeviceStatusHolder(xmlDevice.getName() + "_" + xmlParentDevice.getName(), deviceStatusProvider, registry));
   }

   /**
    * Run through and update all the device statuses
    */
   public void updateDeviceStatusHolders()
   {
      for (int i = 0; i < deviceStatusHolders.size(); i++)
         deviceStatusHolders.get(i).update();
   }
}
