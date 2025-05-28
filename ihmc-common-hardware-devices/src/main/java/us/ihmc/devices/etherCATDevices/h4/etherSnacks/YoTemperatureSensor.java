package us.ihmc.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.devices.TemperatureSensorInterface;
import us.ihmc.devices.YoSensorInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoTemperatureSensor implements YoSensorInterface
{
   private final YoRegistry registry;

   private final YoDouble rawTemperature;
   private final YoDouble temperature;

   private final TemperatureSensorInterface tempSensor;

   public YoTemperatureSensor(String prefix, TemperatureSensorInterface tempSensor, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix+ "TemperatureSensor");

      this.tempSensor = tempSensor;

      rawTemperature = new YoDouble(prefix + "RawTemperature", registry);
      temperature = new YoDouble(prefix + "TemperatureInCelsius", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      rawTemperature.set(tempSensor.getRawTemperature());
      temperature.set(tempSensor.getTemperatureInCelsius());
   }

   public DoubleProvider getRawTemperature()
   {
      return rawTemperature;
   }

   public DoubleProvider getTemperature()
   {
      return temperature;
   }
}
