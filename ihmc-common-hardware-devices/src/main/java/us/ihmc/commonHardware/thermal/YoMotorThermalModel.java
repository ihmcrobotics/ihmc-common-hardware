package us.ihmc.commonHardware.thermal;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoMotorThermalModel
{
   private static final double DEFAULT_TEMPERATURE = 25.0;

   private final MotorThermalModel thermalModel;
   private final YoDouble windingTemperature;
   private final YoDouble housingTemperature;

   public YoMotorThermalModel(String prefix, MotorThermalParameters parameters, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      thermalModel = new MotorThermalModel(parameters, DEFAULT_TEMPERATURE, DEFAULT_TEMPERATURE);
      windingTemperature = new YoDouble(prefix + "_WindingTemperature", registry);
      housingTemperature = new YoDouble(prefix + "_HousingTemperature", registry);

      windingTemperature.set(DEFAULT_TEMPERATURE);
      housingTemperature.set(DEFAULT_TEMPERATURE);

      parentRegistry.addChild(registry);
   }

   public void update(double current, double duration)
   {
      thermalModel.update(current, duration);
      windingTemperature.set(thermalModel.getWindingTemperature());
      housingTemperature.set(thermalModel.getHousingTemperature());
   }

   public void resetTemperature()
   {
      thermalModel.resetTemperature(DEFAULT_TEMPERATURE);
      windingTemperature.set(DEFAULT_TEMPERATURE);
      housingTemperature.set(DEFAULT_TEMPERATURE);
   }
}
