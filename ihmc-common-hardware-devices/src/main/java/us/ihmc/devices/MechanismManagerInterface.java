package us.ihmc.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

public interface MechanismManagerInterface
{
   void initialize();

   void read(LowLevelState measuredJointData);

   void write(JointDesiredOutputReadOnly desiredJointData);

   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   boolean isMotorFaulted();

   default void setIsRobotServoed(boolean isRobotServoed)
   {
   }

   default void onRampingDownMasterGain()
   {
   }

   String getName();
}
