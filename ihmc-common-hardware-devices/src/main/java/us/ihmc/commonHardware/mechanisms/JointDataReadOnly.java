package us.ihmc.commonHardware.mechanisms;

public interface JointDataReadOnly
{
   double getPosition();

   double getVelocity();

   double getAcceleration();

   double getTorque();

   double getStiffness();

   double getDamping();

   default boolean hasStiffness()
   {
      return !Double.isNaN(getStiffness());
   }

   default boolean hasDamping()
   {
      return !Double.isNaN(getDamping());
   }
}
