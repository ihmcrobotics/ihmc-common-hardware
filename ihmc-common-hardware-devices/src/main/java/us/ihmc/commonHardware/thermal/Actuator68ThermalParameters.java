package us.ihmc.commonHardware.thermal;

public class Actuator68ThermalParameters implements ActuatorThermalParameters
{
   @Override
   public double getHousingThermalCapacitance()
   {
      return 557.702088375483; // J/K
   }

   @Override
   public double getWindingThermalCapacitance()
   {
      return 24.2272716001963; // J/K
   }

   @Override
   public double getWindingResistanceAtReferenceTemperature()
   {
      return 0.651; // Ω
   }

   @Override
   public double getReferenceTemperature()
   {
      return 30.4648047818955; // °C
   }

   @Override
   public double getMaxWindingTemperature()
   {
      return 125.0; // °C
   }

   @Override
   public double getHousingToAmbientThermalResistance()
   {
      return 1.5347695708307; // K/W
   }

   @Override
   public double getWindingToHousingThermalResistance()
   {
      return 0.364305635159693; // K/W
   }
}
