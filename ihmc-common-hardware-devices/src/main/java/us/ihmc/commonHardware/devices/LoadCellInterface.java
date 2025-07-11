package us.ihmc.commonHardware.devices;

public interface LoadCellInterface
{
   int getRawVoltage();

   double getVoltage();

   String getName();
}
