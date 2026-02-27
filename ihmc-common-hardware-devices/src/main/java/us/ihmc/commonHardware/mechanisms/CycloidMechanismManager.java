package us.ihmc.commonHardware.mechanisms;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commons.AngleTools;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commonHardware.devices.cycloids.YoCycloidPlatinumTwitter;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.outputData.JointDesiredLoadMode;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.yoVariables.filters.AlphaBasedOnBreakFrequencyProvider;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Map;

/**
 * This class implements the mechanism manager for all cycloids. This converts signals between motor and joint space,
 * including both control and sensing signals.
 *
 * @author Reese Peterson
 */
public class CycloidMechanismManager implements MechanismManagerInterface
{
   private static final double TWO_PI = 2.0 * Math.PI;
   private static final double DEFAULT_TORQUE_BREAK_FREQUENCY = 40.0;
   private static final boolean DEFAULT_PUBLISH_FILTERED_JOINT_STATES = true;
   private static final boolean DEFAULT_USE_FILTERED_JOINT_STATES = false;

   private final String jointName;
   private final YoRegistry registry;
   private final BooleanProvider masterDoPDControlOnTwitter;
   private final YoBoolean doPDControlOnTwitter;

   private YoCycloidPlatinumTwitter platinumTwitter;

   private final YoBoolean publishFilteredJointStates;
   private final YoBoolean useFilteredJointStates;
   private final YoJointData measuredMotorData;
   private final YoJointData desiredMotorData;

   private final YoDouble positionError;
   private final YoDouble velocityError;
   private final YoDouble positionFeedback;
   private final YoDouble velocityFeedback;
   private final YoDouble feedback;
   private final YoDouble velocityFeedbackAlphaVariable;
   private final YoDouble torqueBreakFrequency;
   private final AlphaFilteredYoVariable filteredDesiredTau;

   private final YoJointData measuredActuatorData;
   private final YoJointData desiredActuatorData;

   private final YoDouble yoJointOffset;
   private final YoBoolean updateJointOffset;

   private static final int[] validOffsetIntervals = {-1, 0, 1};
   private final YoLong readTime;
   private final YoLong writeTime;

   private final DoubleProvider time;
   private final YoDouble wakeUpTime;
   private final YoDouble wakeUpDuration;
   private final YoDouble wakeUpPosition;
   private final YoDouble statorTemperature;
   private final YoBoolean isRampingDown;

   private final YoBoolean zeroAgainstLowerLimit;
   private final YoBoolean zeroAgainstUpperLimit;

   private final YoDouble masterGain;

   private DoubleProvider temperatureProvider = this::getTwitterAnalogTemperatureReading;
   private final YoBoolean isStatorAboveShutDownTemperature;
   private final YoBoolean isStatorAboveRecommendedTemperature;

   private final double jointLimitLower;
   private final double jointLimitUpper;
   private boolean disableJointLimitZero = false;

   private final JointLimitTorqueLimiter jointLimitTorqueLimiter;

   public CycloidMechanismManager(double jointOffset,
                                  double jointLimitLower,
                                  double jointLimitUpper,
                                  String jointName,
                                  YoCycloidPlatinumTwitter platinumTwitter,
                                  YoDouble yoTime,
                                  double estimatorDT,
                                  BooleanProvider masterDoPDControlOnTwitter,
                                  double torqueBreakFrequency,
                                  YoRegistry parentRegistry)
   {
      this(jointOffset, jointLimitLower, jointLimitUpper, jointName, platinumTwitter, yoTime, estimatorDT, masterDoPDControlOnTwitter, torqueBreakFrequency,
           DEFAULT_USE_FILTERED_JOINT_STATES, DEFAULT_PUBLISH_FILTERED_JOINT_STATES, parentRegistry);
   }

   /**
    * Initialize the cycloid mechanism manager
    *
    * @param jointOffset          Initial joint offset between actuator 0 and joint 0 in radians
    * @param jointLimitLower      Lower joint limit in radians
    * @param jointLimitUpper      Upper joint limit in radians
    * @param jointName            Name of the joint the cycloid is controlling
    * @param platinumTwitter      The {@code YoCycloidPlatinumTwitter} for the cycloid
    * @param yoTime               The controller time
    * @param estimatorDT          The controller timestep
    * @param masterDoPDControlOnTwitter If true, PD control for position and velocity is done on twitter, else, PD control done in class
    * @param parentRegistry       Parent {@code YoRegistry}
    */
   public CycloidMechanismManager(double jointOffset,
                                  double jointLimitLower,
                                  double jointLimitUpper,
                                  String jointName,
                                  YoCycloidPlatinumTwitter platinumTwitter,
                                  YoDouble yoTime,
                                  double estimatorDT,
                                  BooleanProvider masterDoPDControlOnTwitter,
                                  YoRegistry parentRegistry)
   {
      this(jointOffset,
           jointLimitLower,
           jointLimitUpper,
           jointName,
           platinumTwitter,
           yoTime,
           estimatorDT, masterDoPDControlOnTwitter,
           DEFAULT_TORQUE_BREAK_FREQUENCY,
           parentRegistry);
   }

   /**
    * Initialize the cycloid mechanism manager
    *
    * @param jointOffset          Initial joint offset between actuator 0 and joint 0 in radians
    * @param jointLimitLower      Lower joint limit in radians
    * @param jointLimitUpper      Upper joint limit in radians
    * @param jointName            Name of the joint the cycloid is controlling
    * @param platinumTwitter      The {@code YoCycloidPlatinumTwitter} for the cycloid
    * @param yoTime               The controller time
    * @param estimatorDT          The controller timestep
    * @param masterDoPDControlOnTwitter If true, PD control for position and velocity is done on twitter, else, PD control done in class
    * @param torqueBreakFrequency Initial break frequency for desired torque filtering
    * @param parentRegistry       Parent {@code YoRegistry}
    */
   public CycloidMechanismManager(double jointOffset,
                                  double jointLimitLower,
                                  double jointLimitUpper,
                                  String jointName,
                                  YoCycloidPlatinumTwitter platinumTwitter,
                                  YoDouble yoTime,
                                  double estimatorDT,
                                  BooleanProvider masterDoPDControlOnTwitter,
                                  double torqueBreakFrequency,
                                  boolean useFilteredStates,
                                  boolean publishFilteredStates,
                                  YoRegistry parentRegistry)
   {
      this.time = yoTime;
      this.jointName = jointName;
      this.platinumTwitter = platinumTwitter;
      registry = new YoRegistry(jointName + "_" + getClass().getSimpleName());
      this.masterDoPDControlOnTwitter = masterDoPDControlOnTwitter;

      //joint limits
      this.jointLimitLower = jointLimitLower;
      this.jointLimitUpper = jointLimitUpper;

      //joint offsets
      yoJointOffset = new YoDouble(jointName + "_jointOffset", registry);
      yoJointOffset.set(jointOffset);
      updateJointOffset = new YoBoolean(jointName + "_updateJointOffset", registry);

      measuredMotorData = new YoJointData(jointName + "_MeasuredMotor", false, registry);
      desiredMotorData = new YoJointData(jointName + "_DesiredMotor", true, registry);

      useFilteredJointStates = new YoBoolean(jointName + "_UseFilteredJointStates", registry);
      publishFilteredJointStates = new YoBoolean(jointName + "_PublishFilteredJointStates", registry);
      measuredActuatorData = new YoJointData(jointName + "_MeasuredActuator", false, registry);
      desiredActuatorData = new YoJointData(jointName + "_DesiredActuator", true, registry);
      useFilteredJointStates.set(useFilteredStates);
      publishFilteredJointStates.set(publishFilteredStates);

      positionError = new YoDouble(jointName + "_ActuatorPositionError", registry);
      velocityError = new YoDouble(jointName + "_ActuatorVelocityError", registry);
      positionFeedback = new YoDouble(jointName + "_ActuatorPositionFeedback", registry);
      velocityFeedback = new YoDouble(jointName + "_ActuatorVelocityFeedback", registry);
      feedback = new YoDouble(jointName + "_ActuatorFeedback", registry);
      doPDControlOnTwitter = new YoBoolean(jointName + "_doPDControlOnTwitter", registry);

      velocityFeedbackAlphaVariable = new YoDouble(jointName + "_VelocityFeedbackAlphaVariable", registry);
      velocityFeedbackAlphaVariable.set(1.0);

      this.torqueBreakFrequency = new YoDouble(jointName + "_TorqueBreakFrequency", registry);
      this.torqueBreakFrequency.set(torqueBreakFrequency);
      filteredDesiredTau = new AlphaFilteredYoVariable(jointName + "_DesiredActuatorFilteredTorque",
                                                       registry,
                                                       new AlphaBasedOnBreakFrequencyProvider(this.torqueBreakFrequency, estimatorDT));

      readTime = new YoLong(jointName + "ReadTimeInNanos", registry);
      writeTime = new YoLong(jointName + "WriteTimeInNanos", registry);

      wakeUpTime = new YoDouble(jointName + "_WakeUpTime", registry);
      wakeUpDuration = new YoDouble(jointName + "_WakeUpDuration", registry);
      wakeUpDuration.set(5.0);
      wakeUpPosition = new YoDouble(jointName + "_WakeUpPosition", registry);

      isRampingDown = new YoBoolean(jointName + "_IsRampingDown", registry);

      statorTemperature = new YoDouble(jointName + "_statorTemperature", registry);
      isStatorAboveShutDownTemperature = new YoBoolean(jointName + "_isStatorAboveShutDownTemperature", registry);
      isStatorAboveRecommendedTemperature = new YoBoolean(jointName + "_isStatorAboveRecommendedTemperature", registry);

      masterGain = new YoDouble(jointName + "_MasterGain", registry);

      zeroAgainstLowerLimit = new YoBoolean(jointName + "_ZeroAgainstLowerLimit", registry);
      zeroAgainstUpperLimit = new YoBoolean(jointName + "_ZeroAgainstUpperLimit", registry);

      zeroAgainstLowerLimit.addListener(s ->
                                        {
                                           if (disableJointLimitZero)
                                              LogTools.warn("Attempting to zero " + getName() + " using lower joint limit, but method is disabled for this cycloid");
                                           else if (zeroAgainstLowerLimit.getBooleanValue())
                                              zeroAgainstLimit(jointLimitLower);
                                           zeroAgainstLowerLimit.set(false, false);
                                        });
      zeroAgainstUpperLimit.addListener(s ->
                                        {
                                           if (disableJointLimitZero)
                                              LogTools.warn("Attempting to zero " + getName() + " using lower joint limit, but method is disabled for this cycloid");
                                           else if (zeroAgainstUpperLimit.getBooleanValue())
                                              zeroAgainstLimit(jointLimitUpper);
                                           zeroAgainstUpperLimit.set(false, false);
                                        });

      jointLimitTorqueLimiter = new JointLimitTorqueLimiter(jointName, jointLimitLower, jointLimitUpper, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      // Do Nothing
   }

   @Override
   public void shutDown()
   {
      platinumTwitter.enableDrive(false);
      platinumTwitter.setDesiredMotorTorque(0.0);
      platinumTwitter.setDesiredMotorDamping(0.0);
      platinumTwitter.setDesiredMotorStiffness(0.0);
      platinumTwitter.write();
   }

   @Override
   public void read(Map<String, LowLevelState> measuredJointData)
   {
      read(measuredJointData.get(jointName));
   }

   /**
    * Helper class to read the current state of the cycloid and place the information into the correct {@code LowLevelState}
    *
    * @param measuredJointDataToPack Holder for the measured data from the cycloid
    */
   public void read(LowLevelState measuredJointDataToPack)
   {
      long startTime = System.nanoTime();

      platinumTwitter.read();

      // recompute the joint offset as if this is the zero position for the joint.
      if (updateJointOffset.getBooleanValue())
      {
         updateJointOffset();
         updateJointOffset.set(false);
      }

      measuredMotorData.setPosition(publishFilteredJointStates.getBooleanValue() ?
                                          platinumTwitter.getFilteredMotorPosition() :
                                          platinumTwitter.getMeasuredMotorPosition());
      measuredMotorData.setVelocity(publishFilteredJointStates.getBooleanValue() ?
                                          platinumTwitter.getFilteredMotorVelocity() :
                                          platinumTwitter.getMeasuredMotorVelocity());
      measuredMotorData.setTorque(platinumTwitter.getMeasuredMotorTorque());

      statorTemperature.set(temperatureProvider.getValue());
      if (statorTemperature.getValue() > platinumTwitter.getMaxAllowableStatorTemperature())
      {
         if (!isStatorAboveShutDownTemperature.getValue())
         {
            LogTools.info("OVERHEAT ALERT: " + this.jointName + " stator temperature is " + statorTemperature.getValue() + " deg C. "
                          + "Max allowed stator temperature: " + platinumTwitter.getMaxAllowableStatorTemperature() + " deg C.");
         }
         isStatorAboveShutDownTemperature.set(true);
      }
      if (statorTemperature.getValue() > platinumTwitter.getMaxRecommendedStatorTemperature())
      {
         if (!isStatorAboveRecommendedTemperature.getValue())
         {
            LogTools.info("OVERHEAT WARNING: " + this.jointName + " stator temperature is " + statorTemperature.getValue() + " deg C. "
                          + "Max recommended stator temperature: " + platinumTwitter.getMaxRecommendedStatorTemperature() + " deg C.");
         }
         isStatorAboveRecommendedTemperature.set(true);
      }
      else
      {
         isStatorAboveRecommendedTemperature.set(false);
      }

      double jointPosition = computeJointPosition(publishFilteredJointStates.getBooleanValue() ?
                                                        platinumTwitter.getFilteredOutputPosition() :
                                                        platinumTwitter.getMeasuredOutputPosition(), yoJointOffset.getValue());
      this.measuredActuatorData.setPosition(jointPosition);
      this.measuredActuatorData.setVelocity(publishFilteredJointStates.getBooleanValue() ?
                                                  platinumTwitter.getFilteredOutputVelocity() :
                                                  platinumTwitter.getMeasuredOutputVelocity());
      this.measuredActuatorData.setTorque(platinumTwitter.getMeasuredOutputTorque());

      measuredJointDataToPack.setPosition(this.measuredActuatorData.getPosition());
      measuredJointDataToPack.setVelocity(this.measuredActuatorData.getVelocity());
      measuredJointDataToPack.setEffort(this.measuredActuatorData.getTorque());

      readTime.set(System.nanoTime() - startTime);
   }

   @Override
   public void write(Map<String, JointDesiredOutputBasics> desiredJointData)
   {
      JointDesiredOutputBasics desiredData = desiredJointData.get(jointName);
      desiredData.setMasterGain(masterGain.getDoubleValue());
      write(desiredData);
   }

   /**
    * Helper class to write to specific {@code JointDesiredOutputReadOnly} data to the cycloid
    *
    * @param desiredJointData Holds desired data to be written to cycloid
    */
   public void write(JointDesiredOutputReadOnly desiredJointData)
   {
      long startTime = System.nanoTime();

      double q_d, qd_d, tau_d;
      double stiffness, damping, maxPositionFeedbackError, maxVelocityFeedbackError;
      JointDesiredLoadMode loaded = null;
      stiffness = desiredJointData.hasStiffness() ? desiredJointData.getStiffness() : 0.0;
      damping = desiredJointData.hasDamping() ? desiredJointData.getDamping() : 0.0;

      maxPositionFeedbackError = desiredJointData.hasPositionFeedbackMaxError() ? desiredJointData.getPositionFeedbackMaxError() : Double.POSITIVE_INFINITY;
      maxVelocityFeedbackError = desiredJointData.hasVelocityFeedbackMaxError() ? desiredJointData.getVelocityFeedbackMaxError() : Double.POSITIVE_INFINITY;

      q_d = desiredJointData.hasDesiredPosition() ? desiredJointData.getDesiredPosition() : measuredActuatorData.getPosition();

      qd_d = desiredJointData.hasDesiredVelocity() ? desiredJointData.getDesiredVelocity() : 0.0;
      tau_d = desiredJointData.hasDesiredTorque() ? desiredJointData.getDesiredTorque() : 0.0;

      loaded = desiredJointData.getLoadMode();

      // clamping
      q_d = desiredJointData.hasPositionFeedbackMaxError() ? getClampedDesiredPosition(q_d, measuredActuatorData.getPosition(), maxPositionFeedbackError) : q_d;
      qd_d = desiredJointData.hasVelocityFeedbackMaxError() ? desiredJointData.getClampedDesiredVelocity(measuredActuatorData.getVelocity()) : qd_d;

      double masterGain = MathTools.clamp(this.masterGain.getDoubleValue(), 0.0, 1.0);

      tau_d *= masterGain;
      stiffness *= masterGain;
      damping *= masterGain;

      double timeSinceWakeUp = Math.max(0.0, time.getValue() - wakeUpTime.getValue());

      if (timeSinceWakeUp <= wakeUpDuration.getValue())
      {
         double alpha = MathTools.clamp(timeSinceWakeUp / wakeUpDuration.getValue(), 0.0, 1.0);
         q_d = EuclidCoreTools.interpolate(wakeUpPosition.getValue(), q_d, alpha);
      }

      q_d = MathTools.clamp(q_d, jointLimitLower, jointLimitUpper);

      // Can scale the desired velocity towards zero so velocity feedback is more like viscous damping
      double velocityFeedbackAlpha = MathTools.clamp(velocityFeedbackAlphaVariable.getDoubleValue(), 0.0, 1.0);
      qd_d = InterpolationTools.linearInterpolate(0.0, qd_d, velocityFeedbackAlpha);

      if (!(masterDoPDControlOnTwitter.getValue() || doPDControlOnTwitter.getBooleanValue()))
      {
         double jointPosition = computeJointPosition(useFilteredJointStates.getBooleanValue() ?
                                                           platinumTwitter.getFilteredOutputPosition() :
                                                           platinumTwitter.getMeasuredOutputPosition(), yoJointOffset.getValue());
         double jointVelocity = useFilteredJointStates.getBooleanValue() ? platinumTwitter.getFilteredOutputVelocity() : platinumTwitter.getMeasuredOutputVelocity();

         positionError.set(AngleTools.computeAngleDifferenceMinusPiToPi(q_d, jointPosition));
         velocityError.set(qd_d - jointVelocity);
         positionFeedback.set(stiffness * positionError.getDoubleValue());
         velocityFeedback.set(damping * velocityError.getDoubleValue());
         feedback.set(positionFeedback.getDoubleValue() + velocityFeedback.getDoubleValue());

         tau_d += feedback.getValue();
         q_d = measuredActuatorData.getPosition();
         qd_d = measuredActuatorData.getVelocity();
         stiffness = 0.0;
         damping = 0.0;
      }
      else
      {
         positionError.setToNaN();
         velocityError.setToNaN();
         positionFeedback.setToNaN();
         velocityFeedback.setToNaN();
         feedback.setToNaN();
      }
      if (isRampingDown.getValue()) //TODO(sfasano 20250601) this needs to be fixed (if we even want to keep it)
      {
         double alphaPositionRampDown = 0.05;
         q_d = alphaPositionRampDown * measuredActuatorData.getPosition() + (1.0 - alphaPositionRampDown) * this.desiredActuatorData.getPosition();
         qd_d = 0.0;
      }

      if (jointLimitTorqueLimiter.isTorqueLimitedNearJointLimits())
      {
         tau_d = jointLimitTorqueLimiter.limitDesiredTorques(tau_d, measuredActuatorData.getPosition());
      }

      if (desiredJointData.hasMaxTorque())
         tau_d = MathTools.clamp(tau_d, desiredJointData.getMaxTorque());

      this.desiredActuatorData.setLoadMode(loaded);
      this.desiredActuatorData.setPosition(q_d);
      this.desiredActuatorData.setVelocity(qd_d);
      this.desiredActuatorData.setTorque(tau_d);
      this.desiredActuatorData.setStiffness(stiffness);
      this.desiredActuatorData.setDamping(damping);

      // Low pass filter the desired torque.
      filteredDesiredTau.update(this.desiredActuatorData.getTorque());

      desiredMotorData.setLoadMode(loaded);

      double gearRatio = platinumTwitter.getGearRatio();
      double desiredOutputPosition = computeOutputPosition(this.desiredActuatorData.getPosition(), yoJointOffset.getValue());
      double desiredMotorPosition = computeMotorPosition(desiredOutputPosition, gearRatio, platinumTwitter.getEncoderDifferenceAtOutput());
      double kt = platinumTwitter.getKt();

      desiredMotorData.setPosition(desiredMotorPosition);
      desiredMotorData.setVelocity(this.desiredActuatorData.getVelocity() * gearRatio);
      desiredMotorData.setAcceleration(this.desiredActuatorData.getAcceleration() * gearRatio);
      desiredMotorData.setTorque(filteredDesiredTau.getDoubleValue() / gearRatio);

      // We want to do this because it's way computationally cheaper
      double reflectedMultiplier = 1.0 / (gearRatio * gearRatio * kt);
      desiredMotorData.setStiffness(this.desiredActuatorData.getStiffness() * reflectedMultiplier);
      desiredMotorData.setDamping(this.desiredActuatorData.getDamping() * reflectedMultiplier);

      platinumTwitter.setDesiredMotorPosition(desiredMotorData.getPosition());
      platinumTwitter.setDesiredMotorVelocity(desiredMotorData.getVelocity());
      platinumTwitter.setDesiredMotorTorque(desiredMotorData.getTorque());
      platinumTwitter.setDesiredMotorStiffness(desiredMotorData.getStiffness());
      platinumTwitter.setDesiredMotorDamping(desiredMotorData.getDamping());

      platinumTwitter.setMaxPositionFeedbackError(maxPositionFeedbackError * gearRatio);
      platinumTwitter.setMaxVelocityFeedbackError(maxVelocityFeedbackError * gearRatio);

      platinumTwitter.write();

      writeTime.set(System.nanoTime() - startTime);
   }

   private static double getClampedDesiredPosition(double desiredPosition, double currentPosition, double maxFeedbackError)
   {
      double error = AngleTools.computeAngleDifferenceMinusPiToPi(desiredPosition, currentPosition);

      if (Math.abs(error) > maxFeedbackError)
      {
         double errorClamped = MathTools.clamp(error, maxFeedbackError);
         return AngleTools.trimAngleMinusPiToPi(currentPosition + errorClamped);
      }
      else
      {
         return desiredPosition;
      }
   }

   /**
    * Clear any faults that may be triggered on the cycloid
    */
   public void clearFaults()
   {
      platinumTwitter.clearFaults();
   }

   /**
    * This class is only used as an alternative to the low-level raw encoder zeroing used in {@code YoCycloidPlatinumTwitter}
    */
   @Override
   public void updateJointOffset()
   {
      yoJointOffset.set(-1.0 * platinumTwitter.getMeasuredOutputPosition());
   }

   public void zeroAgainstLimit(double limit)
   {
      platinumTwitter.zeroEncodersWithOffset(limit);
   }

   public void disableJointLimitZeroing(boolean disableJointLimitZero)
   {
      this.disableJointLimitZero = disableJointLimitZero;
   }

   /**
    * @return the current joint offset in radians
    */
   public double getJointOffset()
   {
      return yoJointOffset.getDoubleValue();
   }

   /**
    * Set the joint offset
    *
    * @param jointOffset new joint offset in radians
    */
   public void setJointOffset(double jointOffset)
   {
      yoJointOffset.set(jointOffset);
   }

   @Override
   public void updateActuatorEffortOffsets(TObjectDoubleHashMap<String> effortOffsetMap)
   {
      // TODO Implement me!
   }

   @Override
   public boolean isMotorFaulted()
   {
      return platinumTwitter.isMotorFaulted();
   }

   @Override
   public void setIsRobotServoed(boolean isRobotServoed)
   {
      if (isRobotServoed)
      {
         wakeUpTime.set(time.getValue());
         wakeUpPosition.set(measuredActuatorData.getPosition());
      }

      platinumTwitter.enableDrive(isRobotServoed);
   }

   @Override
   public void setEnableCompensationEfforts(boolean enable)
   {
      platinumTwitter.setEnableCompensationCurrents(enable);
   }

   @Override
   public void setMasterGain(double masterGain)
   {
      this.masterGain.set(masterGain);
   }

   @Override
   public String getName()
   {
      return jointName;
   }

   @Override
   public double getTotalMeasuredMotorCurrent()
   {
      return Math.abs(platinumTwitter.getMeasuredMotorCurrent());
   }

   /**
    * Compute the current joint position based on the cycloid output
    *
    * @param outputPosition Output position of the cycloid in radians
    * @param jointOffset    Offset between cycloid and joint in radians
    * @return Estimated joint position in radians
    */
   private static double computeJointPosition(double outputPosition, double jointOffset)
   {
      return outputPosition + jointOffset;
   }

   /**
    * Compute the current cycloid output position based on the joint
    *
    * @param jointPosition Joint position in radians
    * @param jointOffset   Offset between cycloid and joint in radians
    * @return Estimated cycloid output position in radians
    */
   private static double computeOutputPosition(double jointPosition, double jointOffset)
   {
      return jointPosition - jointOffset;
   }

   /**
    * Compute the motor position based on the output position of the cycloid
    *
    * @param outputPosition                    Cycloid output poisition in radians
    * @param gearRatio                         Gear ratio of the cycloid
    * @param encoderDifferenceAtOutput Difference between motor encoder and output encoder in output space
    * @return Estimated motor position
    */
   private static double computeMotorPosition(double outputPosition, double gearRatio, double encoderDifferenceAtOutput)
   {
      return (outputPosition + encoderDifferenceAtOutput) * gearRatio;
   }

   /**
    * @return The measured temperature of the cycloid in deg Celsius
    */
   public double getTwitterAnalogTemperatureReading()
   {
      return platinumTwitter.getStatorTemperature();
   }

   /**
    * Set a {@code DoubleProvider} to record temperature
    *
    * @param temperatureProvider
    */
   public void setTemperatureProvider(DoubleProvider temperatureProvider)
   {
      this.temperatureProvider = temperatureProvider;
   }

   /**
    * @return True if stator is above the shutdown temp, false otherwise
    */
   public boolean getIsStatorAboveShutDownTemperature()
   {
      return isStatorAboveShutDownTemperature.getBooleanValue();
   }

   /**
    * Set if the stator is above the shutdown temperature
    *
    * @param isStatorAboveShutDownTemperature boolean dictating if the stator is above the shutdown temperature
    */
   public void setIsStatorAboveShutDownTemperature(boolean isStatorAboveShutDownTemperature)
   {
      this.isStatorAboveShutDownTemperature.set(isStatorAboveShutDownTemperature);
   }

   /**
    * @return True if the stator is above recommended temp, false otherwise
    */
   public boolean getIsStatorAboveRecommendedTemperature()
   {
      return isStatorAboveRecommendedTemperature.getValue();
   }

   public JointLimitTorqueLimiter getJointLimitTorqueLimiter()
   {
      return jointLimitTorqueLimiter;
   }

   /**
    * Set if the stator is above the recommended temperature
    *
    * @param isStatorAboveRecommendedTemperature boolean dictating if the stator is above the recommended temperature
    */
   public void setIsStatorAboveRecommendedTemperature(boolean isStatorAboveRecommendedTemperature)
   {
      this.isStatorAboveRecommendedTemperature.set(isStatorAboveRecommendedTemperature);
   }

   public void doPDControlOnTwitter(boolean doPDControlOnTwitter)
   {
      this.doPDControlOnTwitter.set(doPDControlOnTwitter);
   }

   @Override
   public void setPositionBreakFrequency(double breakFrequency)
   {
      platinumTwitter.setOutputPositionBreakFrequency(breakFrequency);
   }

   @Override
   public void setVelocityBreakFrequency(double breakFrequency)
   {
      platinumTwitter.setOutputVelocityBreakFrequency(breakFrequency);
   }

   public void setMotorPositionBreakFrequency(double breakFrequency)
   {
      platinumTwitter.setMotorPositionBreakFrequency(breakFrequency);
   }

   public void setMotorVelocityBreakFrequency(double breakFrequency)
   {
      platinumTwitter.setMotorVelocityBreakFrequency(breakFrequency);
   }
}
