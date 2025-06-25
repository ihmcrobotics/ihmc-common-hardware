package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.sensorProcessing.outputData.ImuData;

public interface IMUManagerInterface
{
   void read(ImuData measuredIMUData);

   String getName();
}
