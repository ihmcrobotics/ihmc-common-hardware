package us.ihmc.commonHardware.thermal;

public class Actuator115ThermalParameters implements ActuatorThermalParameters
{
   @Override
   public double getHousingThermalCapacitance()
   {
      return 100.05153970371; // J/K
   }

   @Override
   public double getWindingThermalCapacitance()
   {
      return 1061.96388275945; // J/K
   }

   @Override
   public double getWindingResistanceAtReferenceTemperature()
   {
      return 0.140; // Ω
   }

   @Override
   public double getReferenceTemperature()
   {
      return 21.9932680531089; // °C
   }

   @Override
   public double getMaxWindingTemperature()
   {
      return 125.0; // °C
   }

   @Override
   public double getHousingToAmbientThermalResistance()
   {
      return 0.418283667825815; // K/W
   }

   @Override
   public double getWindingToHousingThermalResistance()
   {
      return 0.507862723154025; // K/W
   }
}
