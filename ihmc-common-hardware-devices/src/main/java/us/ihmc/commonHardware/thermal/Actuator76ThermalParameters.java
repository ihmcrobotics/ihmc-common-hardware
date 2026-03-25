package us.ihmc.commonHardware.thermal;

public class Actuator76ThermalParameters implements ActuatorThermalParameters
{
   @Override
   public double getHousingThermalCapacitance()
   {
      return 970.338429546758; // J/K
   }

   @Override
   public double getWindingThermalCapacitance()
   {
      return 72.4031172220431; // J/K
   }

   @Override
   public double getWindingResistanceAtReferenceTemperature()
   {
      return 0.579; // Ω
   }

   @Override
   public double getReferenceTemperature()
   {
      return 28.596083444663; // °C
   }

   @Override
   public double getMaxWindingTemperature()
   {
      return 125.0; // °C
   }

   @Override
   public double getHousingToAmbientThermalResistance()
   {
      return 1.19668013600521; // K/W
   }

   @Override
   public double getWindingToHousingThermalResistance()
   {
      return 1.01873749710617; // K/W
   }
}
