package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.LoadCellInterface;

/**
 * Implements the use of an load cell on an EtherSnacks board
 */
public class EtherSnacksLoadCell implements LoadCellInterface
{
   private final String name;

   private double adcConversion;
   private int voltageGain;

   private int rawVoltage;

   /**
    * Create a load cell for an EtherSnacks board. Sets all conversion factors to 1
    *
    * @param name Name of the load cell
    */
   public EtherSnacksLoadCell(String name)
   {
      this(name, 1.0, 1);
   }

   /**
    * Create a load cell for an EtherSnacks board
    *
    * @param name          Name of the load cell
    * @param adcConversion Conversion factor from raw voltage to V
    * @param voltageGain   On-board voltage gain applied to the raw signal
    */
   public EtherSnacksLoadCell(String name, double adcConversion, int voltageGain)
   {
      this.name = name;
      this.adcConversion = adcConversion;
      this.voltageGain = voltageGain;
   }

   public void setADCConversion(double adcConversion)
   {
      this.adcConversion = adcConversion;
   }

   public void setVoltageGain(int voltageGain)
   {
      this.voltageGain = voltageGain;
   }

   public void setRawVoltage(int rawVoltage)
   {
      this.rawVoltage = rawVoltage;
   }

   @Override
   public int getRawVoltage()
   {
      return rawVoltage;
   }

   @Override
   public double getVoltage()
   {
      return ((double) rawVoltage / voltageGain) * adcConversion;
   }

   @Override
   public String getName()
   {
      return name;
   }
}
