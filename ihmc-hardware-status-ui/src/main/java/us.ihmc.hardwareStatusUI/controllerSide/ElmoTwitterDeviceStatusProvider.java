package us.ihmc.hardwareStatusUI.controllerSide;

import us.ihmc.etherCAT.master.Slave;

public interface ElmoTwitterDeviceStatusProvider extends EtherCATDeviceStatusProvider
{
   boolean getIsFaulted();

   boolean getUnderVoltage();

   boolean getOverVoltage();

   boolean getSTODisabled();

   boolean getCurrentShort();

   boolean getOverTemp();

   int getElmoErrorCode();

   double getInputEncoderError();

   double getOutputEncoderError();
}
