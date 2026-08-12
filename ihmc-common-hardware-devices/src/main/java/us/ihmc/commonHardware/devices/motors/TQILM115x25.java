package us.ihmc.commonHardware.devices.motors;

import us.ihmc.commonHardware.thermal.Actuator115ThermalParameters;
import us.ihmc.commonHardware.thermal.Actuator76ThermalParameters;
import us.ihmc.commonHardware.thermal.ActuatorThermalParameters;

public class TQILM115x25 implements MotorParameters
{
   ActuatorThermalParameters thermalParameters = new Actuator115ThermalParameters();

   @Override
   public String getManufacturer()
   {
      return "TQ Robodrive";
   }

   @Override
   public String getModel()
   {
      return "ILM115x25";
   }

   @Override
   public double getKt()
   {
      return 0.281;
   }

   @Override
   public double getResistanceLineToLine()
   {
      return 0.140;
   }

   @Override
   public double getBusVoltage()
   {
      return 48.0;
   }

   @Override
   public double getMaxCurrentContinuous()
   {
      return 14.1;
   }

   @Override
   public double getMaxCurrentPeak()
   {
      return 45.1957295374;
   }

   @Override
   public double getMaxTorqueContinuous()
   {
      return 3.9;
   }

   @Override
   public double getMaxTorquePeak()
   {
      return 12.7;
   }

   @Override
   public double getMaxSpeed()
   {
      return 1400;
   }

   @Override
   public double getPolePairs()
   {
      return 15;
   }

   @Override
   public double getMass()
   {
      return 1.070;
   }

   @Override
   public ActuatorThermalParameters getThermalParameters()
   {
      return thermalParameters;
   }
}
