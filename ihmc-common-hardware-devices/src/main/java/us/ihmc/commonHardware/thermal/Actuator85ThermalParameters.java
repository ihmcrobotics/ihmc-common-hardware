package us.ihmc.commonHardware.thermal;

public class Actuator85ThermalParameters implements ActuatorThermalParameters
{
   @Override
   public double getHousingThermalCapacitance()
   {
      return 467.859628062775; // J/K
   }

   @Override
   public double getWindingThermalCapacitance()
   {
      return 121.607575559668; // J/K
   }

   @Override
   public double getWindingResistanceAtReferenceTemperature()
   {
      return 0.320; // Ω
   }

   @Override
   public double getReferenceTemperature()
   {
      return 22.4915937430372; // °C
   }

   @Override
   public double getMaxWindingTemperature()
   {
      return 125.0; // °C
   }

   @Override
   public double getHousingToAmbientThermalResistance()
   {
      return 0.910279698549021; // K/W
   }

   @Override
   public double getWindingToHousingThermalResistance()
   {
      return 0.264919629290712; // K/W
   }
}
