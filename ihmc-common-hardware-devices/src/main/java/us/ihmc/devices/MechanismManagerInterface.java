package us.ihmc.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.commons.lists.PairList;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

import java.util.Map;

public interface MechanismManagerInterface
{
   void initialize();

   void read(PairList<String, LowLevelState> measuredJointData);

   void write(Map<String, JointDesiredOutputBasics> desiredJointData);

   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   void updateJointOffset();

   boolean isMotorFaulted();

   void setIsRobotServoed(boolean isRobotServoed);

   void setEnableCompensationEfforts(boolean enable);

   void setMasterGain(double masterGain);

   String getName();
}
