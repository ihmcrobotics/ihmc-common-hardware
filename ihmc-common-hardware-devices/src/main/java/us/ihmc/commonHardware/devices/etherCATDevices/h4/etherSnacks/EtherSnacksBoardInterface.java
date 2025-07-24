package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

public interface EtherSnacksBoardInterface
{
   /**
    * Read all sensors attached to the EtherSnacks board
    */
   void readSensors();

   /**
    * Write commands to the EtherSnacks board. Defaulted to do nothing
    */
   default void writeToBoard()
   {
      // Do nothing by default. Most boards don't take any input.
   }
}
