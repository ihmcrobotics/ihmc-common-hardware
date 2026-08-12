package us.ihmc.commonHardware.mechanisms;


import us.ihmc.robotics.outputData.JointDesiredControlMode;
import us.ihmc.robotics.outputData.JointDesiredLoadMode;
import us.ihmc.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoJointDesiredDataHolder
{
   private final YoRegistry registry;

   private final JointDesiredOutputReadOnly jointDesireds;

   private final YoDouble torque;
   private final YoDouble position;
   private final YoDouble velocity;
   private final YoDouble acceleration;

   private final YoDouble stiffness;
   private final YoDouble damping;
   private final YoDouble masterGain;
   private final YoDouble velocityScaling;
   private final YoDouble maxPositionError;
   private final YoDouble maxVelocityError;
   private final YoDouble maxTorque;

   public YoJointDesiredDataHolder(String prefix, JointDesiredOutputReadOnly jointDesireds, YoRegistry parentRegistry)
   {
      this.jointDesireds = jointDesireds;

      registry = new YoRegistry(prefix + this.getClass().getSimpleName());

      position = new YoDouble(prefix + "_position", registry);
      velocity = new YoDouble(prefix + "velocity", registry);
      acceleration = new YoDouble(prefix + "acceleration", registry);
      torque = new YoDouble(prefix + "torque", registry);
      masterGain = new YoDouble(prefix + "masterGain", registry);
      stiffness = new YoDouble(prefix + "stiffness", registry);
      damping = new YoDouble(prefix + "damping", registry);
      maxPositionError = new YoDouble(prefix + "maxPositionError", registry);
      maxVelocityError = new YoDouble(prefix + "maxVelocityError", registry);
      maxTorque = new YoDouble(prefix + "maxTorque", registry);
      velocityScaling = new YoDouble(prefix + "velocityScaling", registry);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      update(jointDesireds);
   }

   public void update(JointDesiredOutputReadOnly jointDesireds)
   {
      position.set(jointDesireds.getDesiredPosition());
      velocity.set(jointDesireds.getDesiredVelocity());
      acceleration.set(jointDesireds.getDesiredAcceleration());
      torque.set(jointDesireds.getDesiredTorque());
      masterGain.set(jointDesireds.getMasterGain());
      stiffness.set(jointDesireds.getStiffness());
      damping.set(jointDesireds.getDamping());
      maxPositionError.set(jointDesireds.getPositionFeedbackMaxError());
      maxVelocityError.set(jointDesireds.getVelocityFeedbackMaxError());
      maxTorque.set(jointDesireds.getMaxTorque());
      velocityScaling.set(jointDesireds.getVelocityScaling());
   }
}
