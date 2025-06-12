package us.ihmc.devices.etherCATDevices.h4.etherSnacks;

public interface EtherSnacksBoardInterface
{
   void readSensors();

   default void write()
   {
      // Do nothing by default. Most boards don't have to write anything.
   }
}
