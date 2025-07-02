package us.ihmc.commonHardware.devices;

public interface IMUInterface
{
   double getRawTemp();

   double getTemp();
   
   double getRawAccelX();

   double getRawAccelY();

   double getRawAccelZ();

   double getAccelX();

   double getAccelY();

   double getAccelZ();

   double getRawGyroX();

   double getRawGyroY();

   double getRawGyroZ();

   double getGyroX();

   double getGyroY();

   double getGyroZ();
   
   String getName();
}
