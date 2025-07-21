package us.ihmc.commonHardware.mechanisms;

import us.ihmc.robotics.outputData.JointDesiredLoadMode;

public class JointData implements JointDataBasics
{
   private double position = 0.0;
   private double velocity = 0.0;
   private double acceleration = 0.0;
   private double torque = 0.0;

   private double stiffness = Double.NaN;
   private double damping = Double.NaN;

   private JointDesiredLoadMode loadMode = null;

   @Override
   public double getPosition()
   {
      return position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }
   
   @Override
   public double getAcceleration()
   {
      return acceleration;
   }

   @Override
   public double getTorque()
   {
      return torque;
   }

   @Override
   public double getStiffness()
   {
      return stiffness;
   }

   @Override
   public double getDamping()
   {
      return damping;
   }

   @Override
   public JointDesiredLoadMode getLoadMode()
   {
      return loadMode;
   }

   @Override
   public void setPosition(double position)
   {
      checkNaN(position);
      this.position = position;
   }

   @Override
   public void setVelocity(double velocity)
   {
      checkNaN(velocity);
      this.velocity = velocity;
   }
   
   @Override
   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
   }

   @Override
   public void setTorque(double torque)
   {
      checkNaN(torque);
      this.torque = torque;
   }

   @Override
   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   @Override
   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   @Override
   public void setLoadMode(JointDesiredLoadMode loadMode)
   {
      this.loadMode = loadMode;
   }

   private void checkNaN(double number)
   {
      //      if (Double.isNaN(number))
      //         throw new RuntimeException("NaN!");
   }
}
