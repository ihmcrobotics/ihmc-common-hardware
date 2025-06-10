package us.ihmc.devices;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoEtherCATDevice
{
   private final YoRegistry registry;

   private final Slave device;
   private final String deviceName;

   private final YoEnum<State> state;
   private final YoBoolean isOperational;
   private final YoBoolean isConfigured;
   private final YoInteger pdiErrorCounter;
   private final YoInteger alStatusCode;
   private final YoInteger ecProcessingUnitErrorCounter;
   private final YoInteger[] rxFrameErrorCounters;
   private final YoInteger[] rxPhysicalLayerErrorCounters;
   private final YoInteger[] lostLinkErrorCounters;

   private final int numPorts;

   public YoEtherCATDevice(Slave device, YoRegistry parentRegistry)
   {
      this.device = device;
      this.deviceName = device.getName();
      this.numPorts = device.getNumberOfPorts();

      registry = new YoRegistry(deviceName + "Status");

      state = new YoEnum<>(deviceName + "_state", registry, State.class);
      isOperational = new YoBoolean(deviceName + "_isOperational", registry);
      isConfigured = new YoBoolean(deviceName + "_isConfigured", registry);
      pdiErrorCounter = new YoInteger(deviceName + "_PDIErrorCounter", registry);
      alStatusCode = new YoInteger(deviceName + "_ALStatusCode", registry);
      ecProcessingUnitErrorCounter = new YoInteger(deviceName + "_processingUnitErrorCounter", registry);

      rxFrameErrorCounters = new YoInteger[numPorts];
      rxPhysicalLayerErrorCounters = new YoInteger[numPorts];
      lostLinkErrorCounters = new YoInteger[numPorts];

      for(int i = 0; i < numPorts; i++)
      {
         String prefix = deviceName + "_" + i;
         rxFrameErrorCounters[i] = new YoInteger(prefix + "_RXErrorFrameCount", registry);
         rxPhysicalLayerErrorCounters[i] = new YoInteger(prefix + "_RXPhysicalLayerErrorCount", registry);
         lostLinkErrorCounters[i] = new YoInteger(prefix + "_LostLinkErrorCount", registry);
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      state.set(device.getState());
      isOperational.set(device.isOperational());
      pdiErrorCounter.set(device.getPDIErrorCounter());
      alStatusCode.set(device.getALStatusCode());
      ecProcessingUnitErrorCounter.set(device.getEthercatProccessingUnitErrorCounter());
      isConfigured.set(device.isConfigured());
      for (int i = 0; i < numPorts; i++)
      {
         rxFrameErrorCounters[i].set(device.getRxFrameErrorCounter(i));
         rxPhysicalLayerErrorCounters[i].set(device.getRxPhysicalLayerErrorCounter(i));
         lostLinkErrorCounters[i].set(device.getLostLinkCounter(i));
      }
   }
}
