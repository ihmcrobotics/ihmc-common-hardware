package us.ihmc.commonHardware.devices.etherCATDevices.h4;

import us.ihmc.commonHardware.devices.genericSensor.YoGenericIMU;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoH4IMU extends YoGenericIMU
{
   private final YoDouble cycleTime;
   private final YoDouble boardTemp;
   private final YoInteger cycleCount;

   private final H4IMU h4IMU;

   /**
    * Create a yovariable wrapper for an H4 IMU
    *
    * @param prefix Prefix applied to all yovariable names
    * @param imu The H4 IMU
    * @param registry Parent registry
    */
   public YoH4IMU(String prefix, H4IMU imu, YoRegistry registry)
   {
      super(prefix, imu, registry);
      this.h4IMU = imu;

      cycleTime = new YoDouble(prefix + "CycleTime", registry);
      boardTemp = new YoDouble(prefix + "BoardTemp", registry);

      cycleCount = new YoInteger(prefix + "CycleCount", registry);
   }

   @Override
   public void update()
   {
      super.update();

      cycleTime.set(h4IMU.getCycleTime());
      boardTemp.set(h4IMU.getBoardTemp());

      cycleCount.set(h4IMU.getCycleCount());
   }
}
