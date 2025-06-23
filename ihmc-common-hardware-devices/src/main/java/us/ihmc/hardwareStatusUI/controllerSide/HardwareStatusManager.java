package us.ihmc.hardwareStatusUI.controllerSide;

import us.ihmc.xmlDescription.devices.AbstractXmlDevice;
import us.ihmc.xmlDescription.devices.AbstractXmlEtherCATDevice;
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

   public void registerDevice(AbstractXmlDevice xmlDevice, DeviceStatusProvider deviceStatusProvider)
   {
      deviceStatusHolders.add(new DeviceStatusHolder(xmlDevice.getName(), deviceStatusProvider, registry));
   }

   public void registerDevice(AbstractXmlEtherCATDevice xmlDevice, EtherCATDeviceStatusProvider deviceStatusProvider)
   {
      deviceStatusHolders.add(new DeviceStatusHolder(xmlDevice.getName(), xmlDevice.getAlias(), xmlDevice.getPosition(), deviceStatusProvider, registry));
   }

   public void registerDevice(AbstractXmlDevice xmlDevice, AbstractXmlDevice xmlParentDevice, EtherCATDeviceStatusProvider deviceStatusProvider)
   {
      deviceStatusHolders.add(new DeviceStatusHolder(xmlDevice.getName() + "_" + xmlParentDevice.getName(), deviceStatusProvider, registry));
   }

   public void updateDeviceStatusHolders()
   {
      for (int i = 0; i < deviceStatusHolders.size(); i ++)
         deviceStatusHolders.get(i).update();
   }

   public ArrayList<DeviceStatusHolder> getDeviceStatusHolders()
   {
      return deviceStatusHolders;
   }
}
