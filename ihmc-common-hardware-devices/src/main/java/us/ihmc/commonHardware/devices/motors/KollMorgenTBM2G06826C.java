package us.ihmc.commonHardware.devices.motors;

import us.ihmc.commonHardware.thermal.Actuator68ThermalParameters;
import us.ihmc.commonHardware.thermal.Actuator76ThermalParameters;
import us.ihmc.commonHardware.thermal.ActuatorThermalParameters;

public class KollMorgenTBM2G06826C implements MotorParameters
{
   ActuatorThermalParameters thermalParameters = new Actuator68ThermalParameters();

   @Override
   public String getManufacturer()
   {
      return "Kollmorgen";
   }

   @Override
   public String getModel()
   {
      return "TBM2G-06826C";
   }

   @Override
   public double getKt()
   {
      return 0.17536248173;
   }

   @Override
   public double getResistanceLineToLine()
   {
      return 0.650996411;
   }

   @Override
   public double getBusVoltage()
   {
      return 48.0;
   }

   @Override
   public double getMaxCurrentContinuous()
   {
      return 6.78594410994;
   }

   @Override
   public double getMaxCurrentPeak()
   {
      return 23.4941930529;
   }

   @Override
   public double getMaxTorqueContinuous()
   {
      return 1.19;
   }

   @Override
   public double getMaxTorquePeak()
   {
      return 4.12;
   }

   @Override
   public double getMaxSpeed()
   {
      return 8000;
   }

   @Override
   public double getPolePairs()
   {
      return 10;
   }

   @Override
   public double getMass()
   {
      return 0.462;
   }

   @Override
   public ActuatorThermalParameters getThermalParameters()
   {
      return thermalParameters;
   }
}
