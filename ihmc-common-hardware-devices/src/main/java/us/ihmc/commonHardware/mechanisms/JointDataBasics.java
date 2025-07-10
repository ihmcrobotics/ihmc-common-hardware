package us.ihmc.commonHardware.mechanisms;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

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
      setLoadMode(jointData.getLoadMode());
   }

   void setPosition(double position);

   void setVelocity(double velocity);
   
   void setAcceleration(double acceleration);

   void setTorque(double torque);

   void setStiffness(double stiffness);

   void setDamping(double damping);

   void setLoadMode(JointDesiredLoadMode loadMode);
}
