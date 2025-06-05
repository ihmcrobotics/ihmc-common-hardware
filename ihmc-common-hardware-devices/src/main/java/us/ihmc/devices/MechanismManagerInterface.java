package us.ihmc.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

import java.util.Map;

public interface MechanismManagerInterface
{
   void initialize();

   void read(LowLevelState measuredJointDataToPack);

   void write(Map<String, JointDesiredOutputBasics> desiredJointData);

   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   boolean isMotorFaulted();

   default void setIsRobotServoed(boolean isRobotServoed)
   {
   }

   default void setMasterGain(double masterGain)
   {
   }

   String getName();
}
