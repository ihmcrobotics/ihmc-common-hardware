package us.ihmc.commonHardware.devices.motors;

import us.ihmc.commonHardware.thermal.ActuatorThermalParameters;

public interface MotorParameters
{
   String getManufacturer();
   String getModel();

   double getKt();
   double getResistanceLineToLine();
   double getBusVoltage();
   double getMaxCurrentContinuous();
   double getMaxCurrentPeak();

   double getMaxTorqueContinuous();
   double getMaxTorquePeak();
   double getMaxSpeed();
   double getPolePairs();
   double getMass();

   ActuatorThermalParameters getThermalParameters();
}
