package us.ihmc.commonHardware.mechanisms;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

import java.util.Map;

public interface MechanismManagerInterface
{
   /**
    * Initialize the manager
    */
   void initialize();

   /**
    * Read data from the mechanism, convert to joint space, and place into the {@code LowLevelState} that corresponds with the joint name
    *
    * @param measuredJointDataMap Map of joint names and joint data
    */
   void read(Map<String, LowLevelState> measuredJointDataMap);

   /**
    * Take desired joint data that corresponds to the joint, convert to mechanism space, and write to the mechanism
    *
    * @param desiredJointDataMap Map of joint names and desired joint data
    */
   void write(Map<String, JointDesiredOutputBasics> desiredJointDataMap);

   /**
    * Shut down the mechanism
    */
   void shutDown();

   /**
    * Update the torque offsets of the mechanism
    *
    * @param effortOffsetMap Map containing the torque offsets
    */
   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   /**
    * Update the joint position offset relative to the mechanism
    */
   void updateJointOffset();

   boolean isMotorFaulted();

   boolean getIsStatorAboveRecommendedTemperature();

   boolean getIsStatorAboveShutDownTemperature();

   double getTotalMeasuredMotorCurrent();

   void clearFaults();

   void setEnableMotors(boolean enableMotors);

   void setEnableCompensationEfforts(boolean enable);

   void setMasterGain(double masterGain);

   void setPositionBreakFrequency(double breakFrequency);

   void setVelocityBreakFrequency(double breakFrequency);

   String getName();

   boolean isDynamicBrakingEnabled();
}
