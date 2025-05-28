package us.ihmc.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.devices.IMUInterface;

public class EtherSnacksIMU implements IMUInterface
{
   private double accelerationConversion, gyroConversion, temperatureScale, temperatureOffset;

   private double rawAccelX, rawAccelY, rawAccelZ;

   private double rawGyroX, rawGyroY, rawGyroZ;

   private double imuTemp;

   private final String name;

   public EtherSnacksIMU(String name)
   {
      this.name = name;
      this.accelerationConversion = 1.0;
      this.gyroConversion = 1.0;
      this.temperatureScale = 1.0;
      this.temperatureOffset = 1.0;
   }

   public EtherSnacksIMU(String name, double accelerationConversion, double gyroConversion, double temperaturScale, double temperatureOffset)
   {
      this.name = name;
      this.accelerationConversion = accelerationConversion;
      this.gyroConversion = gyroConversion;
      this.temperatureScale = temperaturScale;
      this.temperatureOffset = temperatureOffset;
   }

   public void setConversionFactors(double accelerationConversion, double gyroConversion, double temperatureScale, double temperatureOffset)
   {
      this.accelerationConversion = accelerationConversion;
      this.gyroConversion = gyroConversion;
      this.temperatureScale = temperatureScale;
      this.temperatureOffset = temperatureOffset;
   }

   public void setRawAccelerations(double x, double y, double z)
   {
      rawAccelX = x;
      rawAccelY = y;
      rawAccelZ = z;
   }
   public void setRawGyros(double x, double y, double z)
   {
      rawGyroX = x;
      rawGyroY = y;
      rawGyroZ = z;
   }

   public void setRawTemperature(double rawTemperature)
   {
      imuTemp = rawTemperature;
   }

   @Override
   public double getRawTemp()
   {
      return imuTemp;
   }

   @Override
   public double getTemp()
   {
      return imuTemp * temperatureScale + temperatureOffset;
   }

   @Override
   public double getRawAccelX()
   {
      return rawAccelX;
   }

   @Override
   public double getRawAccelY()
   {
      return rawAccelY;
   }

   @Override
   public double getRawAccelZ()
   {
      return rawAccelZ;
   }

   @Override
   public double getAccelX()
   {
      return getRawAccelX() * accelerationConversion;
   }

   @Override
   public double getAccelY()
   {
      return getRawAccelY() * accelerationConversion;
   }

   @Override
   public double getAccelZ()
   {
      return getRawAccelZ() * accelerationConversion;
   }

   @Override
   public double getRawGyroX()
   {
      return rawGyroX;
   }

   @Override
   public double getRawGyroY()
   {
      return rawGyroY;
   }

   @Override
   public double getRawGyroZ()
   {
      return rawGyroZ;
   }

   @Override
   public double getGyroX()
   {
      return getRawGyroX() * gyroConversion;
   }

   @Override
   public double getGyroY()
   {
      return getRawGyroY() * gyroConversion;
   }

   @Override
   public double getGyroZ()
   {
      return getRawGyroZ() * gyroConversion;
   }

   @Override
   public String getName()
   {
      return name;
   }
}
