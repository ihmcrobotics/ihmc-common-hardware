package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.IMUInterface;

/**
 * Implements the use of an IMU on an EtherSnacks board
 */
public class EtherSnacksIMU implements IMUInterface
{
   private double accelerationConversion, gyroConversion, temperatureScale, temperatureOffset;

   private double rawAccelX, rawAccelY, rawAccelZ;

   private double rawGyroX, rawGyroY, rawGyroZ;

   private double imuTemp;

   private final String name;

   /**
    * Create an IMU for an EtherSnacks board. Sets all conversion factors to 1.0
    *
    * @param name Name of the IMU
    */
   public EtherSnacksIMU(String name)
   {
      this(name, 1.0, 1.0, 1.0, 0.0);
   }

   /**
    * Create an IMU for an EtherSnacks board.
    *
    * @param name                   Name of the IMU
    * @param accelerationConversion Conversion factor from raw acceleration to m/s^2
    * @param gyroConversion         Conversion factor from raw gyroscope to rad/s
    * @param temperaturScale        Conversion factor from raw temperature to deg Celsius
    * @param temperatureOffset      Constant temperature offset in deg Celsius
    */
   public EtherSnacksIMU(String name, double accelerationConversion, double gyroConversion, double temperaturScale, double temperatureOffset)
   {
      this.name = name;
      this.accelerationConversion = accelerationConversion;
      this.gyroConversion = gyroConversion;
      this.temperatureScale = temperaturScale;
      this.temperatureOffset = temperatureOffset;
   }

   public void setConversionFactors(double accelerationConversion, double gyroConversion)
   {
      this.setConversionFactors(accelerationConversion, gyroConversion, 1.0, 0.0);
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
