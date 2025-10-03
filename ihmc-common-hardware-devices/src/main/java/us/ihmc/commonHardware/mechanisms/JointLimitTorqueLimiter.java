package us.ihmc.commonHardware.mechanisms;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointLimitTorqueLimiter
{
   private final YoRegistry registry;

   private final YoBoolean limitTorquesNearJointLimits;
   private final YoDouble startingDistanceFromUpperLimit;
   private final YoDouble endingDistanceFromUpperLimit;
   private final YoDouble startingDistanceFromLowerLimit;
   private final YoDouble endingDistanceFromLowerLimit;

   private final YoDouble distanceToUpperLimit;
   private final YoDouble distanceToLowerLimit;

   private final double lowerLimit;
   private final double upperLimit;

   public JointLimitTorqueLimiter(String prefix, double lowerLimit, double upperLimit, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + "JointLimitTorqueLimiter");

      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;

      limitTorquesNearJointLimits = new YoBoolean(prefix + "_LimitTorquesNearJointLimits", registry);
      startingDistanceFromUpperLimit = new YoDouble(prefix + "_StartingDistanceFromUpperLimit", registry);
      endingDistanceFromUpperLimit = new YoDouble(prefix + "_EndingDistanceFromUpperLimit", registry);
      startingDistanceFromLowerLimit = new YoDouble(prefix + "_StartingDistanceFromLowerLimit", registry);
      endingDistanceFromLowerLimit = new YoDouble(prefix + "_EndingDistanceFromLowerLimit", registry);

      distanceToLowerLimit = new YoDouble(prefix + "_DistanceToLowerLimit", registry);
      distanceToUpperLimit = new YoDouble(prefix + "_DistanceToUpperLimit", registry);

      parentRegistry.addChild(registry);
   }

   public double limitDesiredTorques(double desiredTorque, double jointAngle)
   {
      distanceToUpperLimit.set(upperLimit - jointAngle);
      distanceToLowerLimit.set(jointAngle - lowerLimit);
      double numerator = 1.0;
      double denominator = 1.0;
      if (endingDistanceFromLowerLimit.getDoubleValue() > distanceToLowerLimit.getDoubleValue() && desiredTorque < 0.0)
         numerator = 0.0;
      else if (endingDistanceFromUpperLimit.getDoubleValue() > distanceToUpperLimit.getDoubleValue() && desiredTorque > 0.0)
         numerator = 0.0;
      else if (startingDistanceFromLowerLimit.getDoubleValue() > distanceToLowerLimit.getDoubleValue() && desiredTorque < 0.0)
      {
         numerator = distanceToLowerLimit.getDoubleValue() - endingDistanceFromLowerLimit.getDoubleValue();
         denominator = startingDistanceFromLowerLimit.getDoubleValue() - endingDistanceFromLowerLimit.getDoubleValue();
      }
      else if (startingDistanceFromUpperLimit.getDoubleValue() > distanceToUpperLimit.getDoubleValue() && desiredTorque > 0.0)
      {
         numerator = distanceToUpperLimit.getDoubleValue() - endingDistanceFromUpperLimit.getDoubleValue();
         denominator = startingDistanceFromUpperLimit.getDoubleValue() - endingDistanceFromUpperLimit.getDoubleValue();
      }
      return numerator / denominator * desiredTorque;
   }

   public void setLimitTorquesNearJointLimits(boolean limitTorquesNearJointLimits)
   {
      this.limitTorquesNearJointLimits.set(limitTorquesNearJointLimits);
   }

   public void setUpperLimitDistances(double startingDistance, double endingDistance)
   {
      setStartingDistanceFromUpperLimit(startingDistance);
      setEndingDistanceFromUpperLimit(endingDistance);
   }

   public void setLowerLimitDistances(double startingDistance, double endingDistance)
   {
      setStartingDistanceFromLowerLimit(startingDistance);
      setEndingDistanceFromLowerLimit(endingDistance);
   }

   public void setStartingDistanceFromUpperLimit(double startingDistance)
   {
      startingDistanceFromUpperLimit.set(startingDistance);
   }

   public void setEndingDistanceFromUpperLimit(double endingDistance)
   {
      endingDistanceFromUpperLimit.set(endingDistance);
   }

   public void setStartingDistanceFromLowerLimit(double startingDistance)
   {
      startingDistanceFromLowerLimit.set(startingDistance);
   }

   public void setEndingDistanceFromLowerLimit(double endingDistance)
   {
      endingDistanceFromLowerLimit.set(endingDistance);
   }

   public boolean isTorqueLimitedNearJointLimits()
   {
      return limitTorquesNearJointLimits.getBooleanValue();
   }
}
