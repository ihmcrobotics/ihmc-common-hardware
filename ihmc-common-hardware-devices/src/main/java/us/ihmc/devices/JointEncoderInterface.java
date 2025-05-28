package us.ihmc.devices;

public interface JointEncoderInterface
{
   public void update(); 
   
   public double getPosition();

   public double getVelocity();
   
   public boolean isDataValid();
}
