package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;


import us.ihmc.commonHardware.devices.genericIMU.YoGenericIMU;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoEtherSnacksTDKIMU extends YoGenericIMU
{
   private final YoInteger imuStatus;
   private final EtherSnacksTDKIMU tdkIMU;
   public YoEtherSnacksTDKIMU(String prefix, EtherSnacksTDKIMU imu, YoRegistry parentRegistry)
   {
      super(prefix, imu, parentRegistry);
      this.tdkIMU = imu;
      imuStatus = new YoInteger(prefix + "Status", registry);
   }

   @Override
   public void update()
   {
      super.update();
      imuStatus.set(tdkIMU.getRawStatus());
   }
}
