package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.EncoderInterface;

public class EtherSnacksEncoder implements EncoderInterface
{
   private final String name;
   private double conversionToRadians;
   private long rawPosition;

   public EtherSnacksEncoder(String name)
   {
      this(name, 1.0);
   }

   public EtherSnacksEncoder(String name, double conversionToRadians)
   {
      this.name = name;
      this.conversionToRadians = conversionToRadians;
      rawPosition = 0;
   }

   public void setConversionToRadians(double conversionToRadians)
   {
      this.conversionToRadians = conversionToRadians;
   }

   public void setRawPosition(long rawPosition)
   {
      this.rawPosition = rawPosition;
   }

   @Override
   public long getRawPosition()
   {
      return rawPosition;
   }

   @Override
   public double getPosition()
   {
      return rawPosition * conversionToRadians;
   }

   public String getName()
   {
      return name;
   }


}
