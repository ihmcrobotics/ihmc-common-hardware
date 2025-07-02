package us.ihmc.commonHardware.devices;

public interface TemperatureSensorInterface
{
   public double getRawTemperature();
   public void setRawTemperature(double temperature);

   public double getTemperatureInCelsius();
}
