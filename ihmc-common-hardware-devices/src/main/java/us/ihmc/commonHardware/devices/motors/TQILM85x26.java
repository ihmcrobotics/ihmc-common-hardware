package us.ihmc.commonHardware.devices.motors;

import us.ihmc.commonHardware.thermal.Actuator115ThermalParameters;
import us.ihmc.commonHardware.thermal.Actuator85ThermalParameters;
import us.ihmc.commonHardware.thermal.ActuatorThermalParameters;

public class TQILM85x26 implements MotorParameters
{
   ActuatorThermalParameters thermalParameters = new Actuator85ThermalParameters();

   @Override
   public String getManufacturer()
   {
      return "TQ Robodrive";
   }

   @Override
   public String getModel()
   {
      return "ILM85x26";
   }

   @Override
   public double getKt()
   {
      return 0.253;
   }

   @Override
   public double getResistanceLineToLine()
   {
      return 0.320;
   }

   @Override
   public double getBusVoltage()
   {
      return 48.0;
   }

   @Override
   public double getMaxCurrentContinuous()
   {
      return 11.5;
   }

   @Override
   public double getMaxCurrentPeak()
   {
      return 37.1541501976;
   }

   @Override
   public double getMaxTorqueContinuous()
   {
      return 2.9;
   }

   @Override
   public double getMaxTorquePeak()
   {
      return 9.4;
   }

   @Override
   public double getMaxSpeed()
   {
      return 1560;
   }

   @Override
   public double getPolePairs()
   {
      return 10;
   }

   @Override
   public double getMass()
   {
      return 0.670;
   }

   @Override
   public ActuatorThermalParameters getThermalParameters()
   {
      return thermalParameters;
   }
}
