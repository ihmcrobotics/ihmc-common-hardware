package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.TemperatureSensorInterface;

public class EtherSnacksTemperatureSensor implements TemperatureSensorInterface
{
   private double temperatureScale, temperatureOffset;
   private double rawTemperature;

   public EtherSnacksTemperatureSensor()
   {
      temperatureScale = 1.0;
      temperatureOffset = 0.0;
   }

   public EtherSnacksTemperatureSensor(double temperatureScale, double temperatureOffset)
   {
      this.temperatureScale = temperatureScale;
      this.temperatureOffset = temperatureOffset;
   }

   public void setConversionFactors(double temperatureScale, double temperatureOffset)
   {
      this.temperatureScale = temperatureScale;
      this.temperatureOffset = temperatureOffset;
   }

   @Override
   public double getRawTemperature()
   {
      return rawTemperature;
   }

   @Override
   public void setRawTemperature(double temperature)
   {
      rawTemperature = temperature;
   }

   @Override
   public double getTemperatureInCelsius()
   {
      return rawTemperature * temperatureScale + temperatureOffset;
   }
}
