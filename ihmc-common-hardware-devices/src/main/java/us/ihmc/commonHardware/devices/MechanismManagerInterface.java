package us.ihmc.commonHardware.devices;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.commons.lists.PairList;
import us.ihmc.sensorProcessing.outputData.LowLevelState;

import java.util.Map;

public interface MechanismManagerInterface
{
   void initialize();

   void read(Map<String, LowLevelState> measuredJointDataMap);

   void write(Map<String, JointDesiredOutputBasics> desiredJointDataMap);

   void shutDown();

   void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap);

   void updateJointOffset();

   boolean isMotorFaulted();

   boolean getIsStatorAboveRecommendedTemperature();

   boolean getIsStatorAboveShutDownTemperature();

   double getTotalMeasuredMotorCurrent();

   void clearFaults();

   void setIsRobotServoed(boolean isRobotServoed);

   void setEnableCompensationEfforts(boolean enable);

   void setMasterGain(double masterGain);

   String getName();
}
