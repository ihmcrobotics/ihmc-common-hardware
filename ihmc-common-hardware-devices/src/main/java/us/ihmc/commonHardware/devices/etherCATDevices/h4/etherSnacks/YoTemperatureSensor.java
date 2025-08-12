package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.TemperatureSensorInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoTemperatureSensor implements YoSensorInterface
{
   private final YoRegistry registry;

   private final YoDouble rawTemperature;
   private final YoDouble temperature;

   private final TemperatureSensorInterface tempSensor;

   /**
    * Create a yovariable wrapper for a temperature sensor
    *
    * @param prefix         Prefix applied to all yovariable names
    * @param tempSensor     The temperature sensor to be wrapped
    * @param parentRegistry Parent registry
    */
   public YoTemperatureSensor(String prefix, TemperatureSensorInterface tempSensor, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + "TemperatureSensor");

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
