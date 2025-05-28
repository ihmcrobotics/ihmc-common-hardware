package us.ihmc.devices.etherCATDevices.elmo.motorControl;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TwitterControlDesireds
{
   private final YoDouble desiredMotorPosition;
   private final YoDouble desiredMotorVelocity;
   private final YoDouble desiredMotorTorque;
   private final YoDouble desiredFeedForwardCurrent;
   private final YoDouble desiredMotorStiffness;
   private final YoDouble desiredMotorDamping;
   private final YoDouble maxPositionFeedbackError;
   private final YoDouble maxVelocityFeedbackError;

   public TwitterControlDesireds(String prefix, YoRegistry registry)
   {
      desiredMotorPosition = new YoDouble(prefix + "desiredPosition", registry);
      desiredMotorVelocity = new YoDouble(prefix + "desiredVelocity", registry);
      desiredMotorTorque = new YoDouble(prefix + "desiredTorque", registry);
      desiredFeedForwardCurrent = new YoDouble(prefix + "desiredFeedForwardCurrent", registry);

      desiredMotorStiffness = new YoDouble(prefix + "desiredMotorStiffness", registry);
      desiredMotorDamping = new YoDouble(prefix + "desiredMotorDamping", registry);
      maxPositionFeedbackError = new YoDouble(prefix + "maxPositionFeedbackError", registry);
      maxVelocityFeedbackError = new YoDouble(prefix + "maxVelocityFeedbackError", registry);
   }

   public void setDesiredMotorPosition(double qDesired)
   {
      this.desiredMotorPosition.set(qDesired);
   }

   public void setDesiredMotorVelocity(double qdDesired)
   {
      this.desiredMotorVelocity.set(qdDesired);
   }

   public void setDesiredMotorTorque(double tauDesired)
   {
      this.desiredMotorTorque.set(tauDesired);
   }

   public void setDesiredFeedForwardCurrent(double ffCurrent)
   {
      this.desiredFeedForwardCurrent.set(ffCurrent);
   }

   public double getDesiredMotorPosition()
   {
      return desiredMotorPosition.getDoubleValue();
   }

   public double getDesiredMotorVelocity()
   {
      return desiredMotorVelocity.getDoubleValue();
   }

   public double getDesiredMotorTorque()
   {
      return desiredMotorTorque.getDoubleValue();
   }

   public double getDesiredFeedForwardCurrent()
   {
      return desiredFeedForwardCurrent.getDoubleValue();
   }

   public double getDesiredMotorStiffness()
   {
      return desiredMotorStiffness.getDoubleValue();
   }

   public double getDesiredMotorDamping()
   {
      return desiredMotorDamping.getDoubleValue();
   }

   public double getMaxPositionFeedbackError()
   {
      return maxPositionFeedbackError.getDoubleValue();
   }

   public double getMaxVelocityFeedbackError()
   {
      return maxVelocityFeedbackError.getDoubleValue();
   }

   public void setDesiredMotorStiffness(double stiffness)
   {
      desiredMotorStiffness.set(stiffness);
   }

   public void setDesiredMotorDamping(double damping)
   {
      desiredMotorDamping.set(damping);
   }

   public void setMaxPositionFeedbackError(double maxFeedbackError)
   {
      maxPositionFeedbackError.set(maxFeedbackError);
   }

   public void setMaxVelocityFeedbackError(double maxFeedbackError)
   {
      maxVelocityFeedbackError.set(maxFeedbackError);
   }
}
