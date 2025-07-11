package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.sensorProcessing.outputData.ImuData;
import java.util.Map;

public interface IMUManagerInterface
{
   void read(Map<String, ImuData> measuredIMUData);

   String getName();
}
