package us.ihmc.hardwareStatusUI.controllerSide;

public interface ElmoTwitterDeviceStatusProvider extends EtherCATDeviceStatusProvider
{
   boolean isFaulted();

   boolean isUnderVoltage();

   boolean isOverVoltage();

   boolean isSTODisabled();

   boolean isCurrentShort();

   boolean isOverTemp();

   int getElmoErrorCode();

   double getInputEncoderError();

   double getOutputEncoderError();
}
