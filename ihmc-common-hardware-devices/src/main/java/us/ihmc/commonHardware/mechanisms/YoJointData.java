package us.ihmc.commonHardware.mechanisms;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointData implements JointDataBasics
{
   private static final boolean FAIL_ON_NAN = false;

   private final YoDouble position;
   private final YoDouble velocity;
   private final YoDouble acceleration;
   private final YoDouble torque;
   private final YoDouble stiffness;
   private final YoDouble damping;

   public YoJointData(String prefix, boolean createGainVariables, YoRegistry registry)
   {
      String positionName = "Position";
      String velocityName = "Velocity";
      String accelerationName = "Acceleration";
      String torqueName = "Torque";
      String stiffnessName = "Kp";
      String dampingName = "Kd";

      if (prefix != null)
      {
         positionName = prefix + positionName;
         velocityName = prefix + velocityName;
         accelerationName = prefix + accelerationName;
         torqueName = prefix + torqueName;
         stiffnessName = prefix + stiffnessName;
         dampingName = prefix + dampingName;
      }

      position = new YoDouble(positionName, registry);
      velocity = new YoDouble(velocityName, registry);
      torque = new YoDouble(torqueName, registry);
      if (createGainVariables)
      {
         stiffness = new YoDouble(stiffnessName, registry);
         damping = new YoDouble(dampingName, registry);
         acceleration = new YoDouble(accelerationName, registry);
      }
      else
      {
         stiffness = null;
         damping = null;
         acceleration = null;
      }
   }

   @Override
   public double getPosition()
   {
      return position.getValue();
   }

   @Override
   public double getVelocity()
   {
      return velocity.getValue();
   }

   @Override
   public double getAcceleration()
   {
      return acceleration.getValue();
   }

   @Override
   public double getTorque()
   {
      return torque.getValue();
   }

   @Override
   public boolean hasStiffness()
   {
      return stiffness != null && JointDataBasics.super.hasStiffness();
   }

   @Override
   public double getStiffness()
   {
      return stiffness.getValue();
   }

   @Override
   public boolean hasDamping()
   {
      return damping != null && JointDataBasics.super.hasDamping();
   }

   @Override
   public double getDamping()
   {
      return damping.getValue();
   }

   @Override
   public void setPosition(double position)
   {
      checkNaN(position);
      this.position.set(position);
   }

   @Override
   public void setVelocity(double velocity)
   {
      checkNaN(velocity);
      this.velocity.set(velocity);
   }

   @Override
   public void setAcceleration(double acceleration)
   {
      checkNaN(acceleration);
      this.acceleration.set(acceleration);
   }

   @Override
   public void setTorque(double torque)
   {
      checkNaN(torque);
      this.torque.set(torque);
   }

   @Override
   public void setStiffness(double stiffness)
   {
      if (this.stiffness != null)
         this.stiffness.set(stiffness);
   }

   @Override
   public void setDamping(double damping)
   {
      if (this.damping != null)
         this.damping.set(damping);
   }

   private void checkNaN(double number)
   {
      if (FAIL_ON_NAN)
      {
         if (Double.isNaN(number))
            throw new RuntimeException("NaN!");
      }
   }

   @Override
   public String toString()
   {
      return "YoJointData [position=" + position + ", velocity=" + velocity + ", torque=" + torque + "]";
   }
}
