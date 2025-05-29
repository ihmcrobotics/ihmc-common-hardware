package us.ihmc.devices.genericIMU;

import us.ihmc.sensorProcessing.outputData.ImuData;

public interface IMUManagerInterface
{
   public void read(ImuData measuredIMUData);
}
