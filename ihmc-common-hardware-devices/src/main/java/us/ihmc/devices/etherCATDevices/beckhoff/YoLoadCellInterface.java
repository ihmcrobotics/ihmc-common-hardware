package us.ihmc.devices.etherCATDevices.beckhoff;

import us.ihmc.yoVariables.variable.YoDouble;

public interface YoLoadCellInterface
{
   public void read();
   
   public double getForce();
   
   public YoDouble getYoForce();
}
