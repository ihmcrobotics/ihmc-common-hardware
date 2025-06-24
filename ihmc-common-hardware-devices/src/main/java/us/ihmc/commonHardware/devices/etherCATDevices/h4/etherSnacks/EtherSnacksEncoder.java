package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.JointEncoderInterface;

public class EtherSnacksEncoder
{
   private final String name;
   private double conversionToRadians;
   private double rawPosition;

   public EtherSnacksEncoder(String name)
   {
      this(name, 1.0);
   }

   public EtherSnacksEncoder(String name, double conversionToRadians)
   {
      this.name = name;
      this.conversionToRadians = conversionToRadians;
      rawPosition = 0.0;
   }

   public void setConversionToRadians(double conversionToRadians)
   {
      this.conversionToRadians = conversionToRadians;
   }

   public void setRawPosition(double rawPosition)
   {
      this.rawPosition = rawPosition;
   }

   public double getRawPosition()
   {
      return rawPosition;
   }

   public double getPosition()
   {
      return rawPosition * conversionToRadians;
   }

   public String getName()
   {
      return name;
   }


}
