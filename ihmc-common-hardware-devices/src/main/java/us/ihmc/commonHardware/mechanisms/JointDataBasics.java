package us.ihmc.commonHardware.mechanisms;

/**
 * This interface allows for placing all joint data in a common location, including position, velocity, acceleration,torque,
 * stiffness, damping, and the current load mode
 */
public interface JointDataBasics extends JointDataReadOnly
{
   default void set(JointDataReadOnly jointData)
   {
      setPosition(jointData.getPosition());
      setVelocity(jointData.getVelocity());
      setAcceleration(jointData.getAcceleration());
      setTorque(jointData.getTorque());
      setStiffness(jointData.getStiffness());
      setDamping(jointData.getDamping());
   }

   void setPosition(double position);

   void setVelocity(double velocity);
   
   void setAcceleration(double acceleration);

   void setTorque(double torque);

   void setStiffness(double stiffness);

   void setDamping(double damping);
}
