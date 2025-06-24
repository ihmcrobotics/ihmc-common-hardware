package us.ihmc.devices.etherCATDevices.h4.etherSnacks;

public interface EtherSnacksBoardInterface
{
   void readSensors();

   default void writeToBoard()
   {
      // Do nothing by default. Most boards don't take any input.
   }
}
