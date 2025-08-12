package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.EncoderInterface;

/**
 * Implements the use of an encoder on an EtherSnacks board
 */
public class EtherSnacksEncoder implements EncoderInterface
{
   private final String name;
   private double conversionToRadians;
   private long rawPosition;

   /**
    * Create an encoder for an EtherSnacks board. Initializes conversion factor to 1.0
    *
    * @param name Name of the encoder
    */
   public EtherSnacksEncoder(String name)
   {
      this(name, 1.0);
   }

   /**
    * Create an encoder for an EtherSnacks board
    *
    * @param name                Name of the encoder
    * @param conversionToRadians The conversion factor from raw position to radians
    */
   public EtherSnacksEncoder(String name, double conversionToRadians)
   {
      this.name = name;
      this.conversionToRadians = conversionToRadians;
      rawPosition = 0L;
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

   @Override
   public String getName()
   {
      return name;
   }
}
