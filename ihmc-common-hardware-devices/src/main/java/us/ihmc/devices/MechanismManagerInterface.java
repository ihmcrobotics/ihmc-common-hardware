package us.ihmc.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

import java.util.Map;

public interface MechanismManagerInterface
{
   void initialize();

   void read(LowLevelState measuredJointDataToPack);

   void write(Map<String, JointDesiredOutputBasics> desiredJointData);

   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   void updateJointOffset();

   boolean isMotorFaulted();

   default void setIsRobotServoed(boolean isRobotServoed)
   {
   }

   default void setMasterGain(double masterGain)
   {
   }

   String getName();
}
