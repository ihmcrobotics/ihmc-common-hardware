package us.ihmc.commonHardware.devices.motors;

import us.ihmc.commonHardware.thermal.Actuator76ThermalParameters;
import us.ihmc.commonHardware.thermal.ActuatorThermalParameters;

public class KollMorgenTBM2G07626C implements MotorParameters
{
   ActuatorThermalParameters thermalParameters = new Actuator76ThermalParameters();

   @Override
   public String getManufacturer()
   {
      return "Kollmorgen";
   }

   @Override
   public String getModel()
   {
      return "BM2G-07626C";
   }

   @Override
   public double getKt()
   {
      return 0.21354624791;
   }

   @Override
   public double getResistanceLineToLine()
   {
      return 0.579;
   }

   @Override
   public double getBusVoltage()
   {
      return 48.0;
   }

   @Override
   public double getMaxCurrentContinuous()
   {
      return 7.49252218505;
   }

   @Override
   public double getMaxCurrentPeak()
   {
      return 26.5047972296;
   }

   @Override
   public double getMaxTorqueContinuous()
   {
      return 1.60;
   }

   @Override
   public double getMaxTorquePeak()
   {
      return 5.66;
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
      return 0.596;
   }

   @Override
   public ActuatorThermalParameters getThermalParameters()
   {
      return thermalParameters;
   }
}
