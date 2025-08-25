package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.sensorProcessing.outputData.ImuData;

import java.util.Map;

public interface IMUManagerInterface
{
   /**
    * Reads the data from the IMU and places into the corresponding {@code ImuData}
    *
    * @param measuredIMUData Map with imu names and data
    */
   void read(Map<String, ImuData> measuredIMUData);

   String getName();
}
