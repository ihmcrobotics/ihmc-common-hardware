package us.ihmc.devices.genericIMU;

import us.ihmc.devices.IMUInterface;

public class LowLevelIMUManager
{
   private final IMUInterface imu;

   private final double[] linearAcceleration = new double[3];
   private final double[] angularVelocity = new double[3];
   private double temperature;


   public LowLevelIMUManager(IMUInterface imu)
   {
      this.imu = imu;
   }

   public void read()
   {
      linearAcceleration[0] = imu.getAccelX();
      linearAcceleration[1] = imu.getAccelY();
      linearAcceleration[2] = imu.getAccelZ();

      angularVelocity[0] = imu.getGyroX();
      angularVelocity[1] = imu.getGyroY();
      angularVelocity[2] = imu.getGyroZ();

      temperature = imu.getTemp();
   }

   public double[] getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public double[] getAngularVelocity()
   {
      return angularVelocity;
   }

   public double getTemperature()
   {
      return temperature;
   }

   public String getSensorName()
   {
      return imu.getName();
   }
}
