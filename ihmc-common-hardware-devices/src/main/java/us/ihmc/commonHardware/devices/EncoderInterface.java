package us.ihmc.commonHardware.devices;

public interface EncoderInterface
{
   long getRawPosition();
   
   double getPosition();

   String getName();
}
