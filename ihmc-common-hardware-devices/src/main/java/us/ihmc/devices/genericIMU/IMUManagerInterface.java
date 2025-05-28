package us.ihmc.devices.genericIMU;

import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

public interface IMUManagerInterface
{
   public void update(SensorDataContext sensorDataContext);
}
