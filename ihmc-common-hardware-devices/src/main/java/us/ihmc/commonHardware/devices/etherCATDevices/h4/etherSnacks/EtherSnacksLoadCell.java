package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.LoadCellInterface;

public class EtherSnacksLoadCell implements LoadCellInterface
{
   private final String name;

   private double adcConversion;
   private int voltageGain;

   private int rawVoltage;

   public EtherSnacksLoadCell(String name)
   {
      this(name, 1.0, 1);
   }

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

   public int getRawVoltage()
   {
      return rawVoltage;
   }

   public double getVoltage()
   {
      return ((double) rawVoltage / voltageGain) * adcConversion;
   }
}
