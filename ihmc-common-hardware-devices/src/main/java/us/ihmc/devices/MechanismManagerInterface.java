package us.ihmc.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

public interface MechanismManagerInterface
{
   public void initialize();

   public void read(LowLevelState measuredJointData);

   public void write(JointDesiredOutputReadOnly desiredJointData);

   public void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   public default void setIsRobotServoed(boolean isRobotServoed)
   {
   }

   public default void onRampingDownMasterGain()
   {
   }

   String getName();
}
