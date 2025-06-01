package us.ihmc.devices.cycloids;

import us.ihmc.devices.cycloids.CycloidActuatorPackage;
import us.ihmc.devices.cycloids.CycloidActuatorParameters;
import us.ihmc.devices.cycloids.CycloidPlatinumTwitter;
import us.ihmc.devices.cycloids.SILParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.devices.etherCATDevices.elmo.ElmoTwitterStatusRegisterProcessor;
import us.ihmc.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.etherCAT.slaves.DSP402Slave;
import us.ihmc.etherCAT.slaves.DSP402Slave.StatusWord;
import us.ihmc.etherCAT.slaves.elmo.ElmoErrorCodes;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoCycloidPlatinumTwitter implements YoGenericTwitter
{
   //The controller will try to reenable the drive if this is true, this can be scary on real hardware
   private static final boolean CLEAR_FAULTS = true;

   // Current variables
   private static final double CURRENT_SIGNAL_RANGE = 1000.0;

   //Raw Velocity to Rad/s
   private static final double RAW_VELOCITY_TO_COUNTS_PER_SEC = 10000.0;

   // RTD 1000 temperature sensor function coefficients
   private static final double[] TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS = new double[] {10.325581, 224.7863, -360.157212};

   private final double dt;
   private final String name;
   private final YoRegistry registry;

   private final DoubleProvider time;
   private final YoDouble previousTime;
   private final YoDouble estimatedDt;

   private final CycloidPlatinumTwitter platinumTwitter;

   // conversion variables
   private final double motorEncoderCountsToMotorRadians;
   private final double motorEncoderCountsToOutputRadians;

   private final double outputEncoderCountsToOutputRadians;
   private final double outputRadiansToMotorEncoderCounts;

   // command variables
   protected final YoDouble desiredMotorPosition;
   protected final YoDouble desiredMotorVelocity;
   protected final YoDouble desiredFeedForwardMotorCurrent;
   protected final YoDouble desiredMotorCurrent;
   protected final YoDouble desiredMotorCurrentWithFF;
   protected final YoDouble desiredMotorTorque;
   protected final YoDouble desiredOutputTorque;

   protected final YoInteger rawDesiredMotorPosition;
   protected final YoInteger rawDesiredMotorVelocity;
   protected final YoInteger rawDesiredMotorEffortPercentage;

   protected final YoBoolean[] digitalOutputs = new YoBoolean[6];
   protected final YoEnum<ElmoModeOfOperation> requestedModeOfOperation;
   protected final YoEnum<ElmoModeOfOperation> currentModeOfOperation;

   // measured variables
   private final YoDouble measuredMotorPosition;
   private final YoDouble measuredMotorVelocity;
   private final YoDouble measuredMotorVelocityFD;
   private final YoBoolean useFDforMotorVelocity;

   private final YoDouble filteredVelocityAlphaValue;
   private final AlphaFilteredYoVariable preFilteredMotorVelocity, filteredMotorVelocity;

   private final YoDouble measuredMotorPositionInOutput;
   private final YoDouble measuredMotorVelocityInOutput;

   private final YoDouble measuredOutputPosition;
   private final YoDouble measuredOutputVelocity;
   private final YoDouble measuredOutputVelocityFD;
   private final YoBoolean useFDforOutputVelocity;

   private final YoDouble filteredOutputVelocityAlphaValue;
   private final AlphaFilteredYoVariable preFilteredOutputVelocity, filteredOutputVelocity;

   private final YoLong maxDriveCurrentMilliAmps;
   private final YoDouble measuredMotorCurrent;
   private final YoDouble estimatedMotorTorque;
   private final YoDouble estimatedOutputTorque;

   private final YoInteger rawMeasuredMotorPosition;
   private final YoDouble rawMeasuredMotorVelocity;
   private final YoDouble rawMeasuredOuputPosition;
   private final YoDouble rawMeasuredOutputVelocity;
   private final YoInteger rawMeasuredMotorCurrent;

   private final YoEnum<DSP402Slave.StatusWord> statusWord;
   private final YoLong elmoStatusRegister;
   private final YoInteger errorCode;
   private final YoEnum<?> elmoErrorString;
   private final YoDouble measuredBusVoltage;
   private final YoDouble measuredAnalogInput2;

   // error variables
   private final ElmoTwitterStatusRegisterProcessor statusRegisterProcessor;
   private final YoBoolean DRIVE_FAULTED;
   private final YoBoolean UNDER_VOLTAGE;
   private final YoBoolean OVER_VOLTAGE;
   private final YoBoolean STO_DISABLED;
   private final YoBoolean CURRENT_SHORT;
   private final YoBoolean OVER_TEMPERATURE;
   private final YoBoolean MOTOR_ENABLED;
   private final YoBoolean MOTOR_FAULT;
   private final YoBoolean CURRENT_LIMITED;

   // SIL Tunable Variables
   private final YoBoolean saveR2ToNVM; //Save R2 to NVM, this can only be done once per power cycle

   private final YoDouble dahlFrictionForce; // Dahl Friction Force
   private final YoDouble dahlSlope; // Dahl Slope (offset by + 1.0)
   private final YoDouble linearDampingCompensation; // Linear Damping Compensation
   private final YoDouble dahlOutputScalar; // Dahl Output Scalar
   private final YoDouble linearDampingOutputScalar; // Linear Damping Output Scalar
   private final YoDouble coggingOutputScalar; // Cogging Output Scalar

   // SIL Acceleration Integration Variables
   private final YoDouble accelerationIntegrationDesiredMotorPosition;
   private final YoDouble accelerationIntegrationDesiredMotorVelocity;
   private YoDouble accelerationIntegrationStiffness;
   private YoDouble accelerationIntegrationDamping;
   private YoDouble accelerationIntegrationScalar;
   private YoDouble accelerationIntegrationMaxPositionError;
   private YoDouble accelerationIntegrationMaxVelocityError;

   // Control variables
   private final YoBoolean enableDrive;
   private final YoBoolean enableCompensation;
   private final YoBoolean clearFaults;
   private final YoBoolean stayDisabled;
   private final YoEnum<DSP402Slave.ControlWord> controlWord;

   private final YoDouble kt;
   private final YoInteger maxAllowableStatorTemperature;
   private final YoInteger maxRecommendedStatorTemperature;
   private final YoDouble motorDirection;
   private final YoDouble gearRatio;

   private final CycloidActuatorParameters actuatorParameters;
   private final SILParameters silParameters;

   // SIL Debuggging variables
   private final YoDouble sil_linearDampingCompensationCurrent;
   private final YoDouble sil_coggingCompensationMotorCurrent;
   private final YoDouble sil_dahlFrictionCompensationCurrent;
   private final YoDouble sil_acceleratiojnIntegrationMotorFeedbackCurrent;
   private final YoDouble sil_accelerationIntegrationMeasuredMotorPosition;
   private final YoDouble sil_accelerationIntegrationMeasuredMotorVelocity;

   // SIL Socket warning and error status variables
   private final YoDouble inputEncoderWarningValue;
   private final YoDouble inputEncoderErrorValue;
   private final YoDouble outputEncoderWarningValue;
   private final YoDouble outputEncoderErrorValue;

   private YoEnum<EncoderState> inputEncoderState;
   private YoEnum<EncoderState> outputEncoderState;

   private final CycloidActuatorPackage actuatorPackage;

   private final YoBoolean reverseMotorDirection;
//   private double motorDirection = 1.0;
   private final YoDouble zeroPositionOffset;

   private enum EncoderState
   {
      NORMAL, WARNING, ERROR
   }

   public YoCycloidPlatinumTwitter(String prefix,
                                   CycloidPlatinumTwitter twitter,
                                   DoubleProvider time,
                                   CycloidActuatorPackage actuatorPackage,
                                   boolean isMotorDirectionReversed,
                                   double zeroPositionOffset,
                                   double dt,
                                   YoRegistry parentRegistry)
   {
      this.time = time;
      this.dt = dt;
      this.platinumTwitter = twitter;
      this.actuatorPackage = actuatorPackage;
      name = prefix + getClass().getSimpleName();
      registry = new YoRegistry(name);

      this.actuatorParameters = CycloidActuatorParameters.createCycloidParameters(actuatorPackage);
      this.silParameters = SILParameters.createParameters(actuatorPackage);

      this.reverseMotorDirection = new YoBoolean(name + "MotorDirectionIsReversed", registry);
      this.reverseMotorDirection.set(isMotorDirectionReversed);
      this.zeroPositionOffset = new YoDouble(name + "ZeroPositionOffset", registry);
      this.zeroPositionOffset.set(zeroPositionOffset);
      motorDirection = new YoDouble(prefix + "motorDirection", registry);
      setMotorDirection(isMotorDirectionReversed);

      motorDirection.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable yoVariable)
         {
            if (Math.abs(motorDirection.getDoubleValue()) != 1.0)
            {
               double attemptedValue = motorDirection.getDoubleValue();
               setMotorDirection(reverseMotorDirection.getBooleanValue());
               double actualValue = motorDirection.getDoubleValue();
               LogTools.warn("Attempted to set motor direction to " + attemptedValue + " which is not allowed. Motor direction can only be 1.0 or -1.0, resetting to " + actualValue);
            }
         }
      });

      this.reverseMotorDirection.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable yoVariable)
         {
            setMotorDirection(reverseMotorDirection.getBooleanValue());
         }
      });

      previousTime = new YoDouble(prefix + "PreviousTime", registry);
      estimatedDt = new YoDouble(prefix + "EstimatedDt", registry);
      previousTime.setToNaN();
      estimatedDt.setToNaN();

      this.kt = new YoDouble(prefix + "kt", registry);
      this.kt.set(actuatorParameters.getKt());

      this.maxAllowableStatorTemperature = new YoInteger(prefix + "maxAllowableStatorTemperature", registry);
      this.maxRecommendedStatorTemperature = new YoInteger(prefix + "maxRecommendedStatorTemperature", registry);

      gearRatio = new YoDouble(prefix + "gearRatio", registry);
      gearRatio.set(actuatorParameters.getGearRatio());

      motorEncoderCountsToMotorRadians = (2.0 * Math.PI) / actuatorParameters.getCountsPerMotorRevolution();
      motorEncoderCountsToOutputRadians = motorEncoderCountsToMotorRadians * actuatorParameters.getGearRatio();

      outputRadiansToMotorEncoderCounts = 1.0 / (motorEncoderCountsToOutputRadians);
      outputEncoderCountsToOutputRadians = (2.0 * Math.PI) / actuatorParameters.getCountsPerOutputRevolution();

      maxDriveCurrentMilliAmps = new YoLong(prefix + "MaxDriveCurrentMilliAmps", registry);

      //desireds
      desiredMotorPosition = new YoDouble(prefix + "desiredMotorPosition", registry);
      desiredMotorVelocity = new YoDouble(prefix + "desiredMotorVelocity", registry);
      desiredFeedForwardMotorCurrent = new YoDouble(prefix + "desiredFeedForwardMotorCurrent", registry);
      desiredMotorCurrent = new YoDouble(prefix + "desiredMotorCurrent", registry);
      desiredMotorCurrentWithFF = new YoDouble(prefix + "desiredMotorCurrentWithFF", registry);
      desiredMotorTorque = new YoDouble(prefix + "desiredMotorTorque", registry);
      desiredOutputTorque = new YoDouble(prefix + "desiredOutputTorque", registry);

      //desired in raw units
      rawDesiredMotorPosition = new YoInteger(prefix + "rawDesiredMotorPosition", registry);
      rawDesiredMotorVelocity = new YoInteger(prefix + "rawDesiredMotorVelocity", registry);
      rawDesiredMotorEffortPercentage = new YoInteger(prefix + "rawDesiredMotorEffortPercentage", registry);

      for (int i = 0; i < 6; i++)
      {
         digitalOutputs[i] = new YoBoolean(prefix + "digitalOutput_" + i, registry);
      }

      // SIL Tunable Variables
      saveR2ToNVM = new YoBoolean(prefix + "saveR2ToNVM", registry);
      enableCompensation = new YoBoolean(prefix + "enableCompensationCurrents", registry);
      dahlFrictionForce = new YoDouble(prefix + "dahlFrictionForce", registry); // Dahl Friction Force
      dahlSlope = new YoDouble(prefix + "dahlSlope", registry); // Dahl Slope (offset by + 1.0)                         
      linearDampingCompensation = new YoDouble(prefix + "linearDampingCompensation", registry); // Linear Damping Compensation
      // Do not allow compensation values below 0
      applyValueLimits(dahlFrictionForce, 0.0, Double.POSITIVE_INFINITY);
      applyValueLimits(dahlSlope, 0.0, Double.POSITIVE_INFINITY);
      applyValueLimits(linearDampingCompensation, 0.0, Double.POSITIVE_INFINITY);

      dahlOutputScalar = new YoDouble(prefix + "dahlOutputScalar", registry); // Dahl Output Scalar                            
      linearDampingOutputScalar = new YoDouble(prefix + "linearDampingOutputScalar", registry); // Linear Damping Output Scalar         
      coggingOutputScalar = new YoDouble(prefix + "coggingOutputScalar", registry); // Cogging Output Scalar
      //Compensation Scalars should only be between 0 and 1
      applyValueLimits(dahlOutputScalar, 0.0, 1.0);
      applyValueLimits(linearDampingOutputScalar, 0.0, 1.0);
      applyValueLimits(coggingOutputScalar, 0.0, 1.0);

      dahlFrictionForce.set(silParameters.getDahlFrictionForceGain());
      dahlSlope.set(silParameters.getDahlFrictionSlope());
      linearDampingCompensation.set(silParameters.getLinearDampingCompensationGain());

      enableCompensation.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            if (enableCompensation.getBooleanValue())
            {
               coggingOutputScalar.set(silParameters.getCoggingOutputScalar());
               dahlOutputScalar.set(silParameters.getDahlOutputScalar());
               linearDampingOutputScalar.set(silParameters.getLinearDampingOutputScalar());
            }
            else
            {
               coggingOutputScalar.set(0.0);
               dahlOutputScalar.set(0.0);
               linearDampingOutputScalar.set(0.0);
            }
         }
      });

      // SIL Acceleration Integration Variables
      accelerationIntegrationDesiredMotorPosition = new YoDouble(prefix + "AccelerationIntegration_DesiredMotorPosition", registry);
      accelerationIntegrationDesiredMotorVelocity = new YoDouble(prefix + "AccelerationIntegration_DesiredMotorVelocity", registry);
      accelerationIntegrationStiffness = new YoDouble(prefix + "AccelerationIntegration_Stiffness", registry);
      accelerationIntegrationDamping = new YoDouble(prefix + "AccelerationIntegration_Damping", registry);
      accelerationIntegrationScalar = new YoDouble(prefix + "AccelerationIntegration_Scalar", registry);
      accelerationIntegrationMaxPositionError = new YoDouble(prefix + "AccelerationIntegration_MaxPositionError", registry);
      accelerationIntegrationMaxVelocityError = new YoDouble(prefix + "AccelerationIntegration_MaxVelocityError", registry);

      // SIL debugging variables
      sil_linearDampingCompensationCurrent = new YoDouble(prefix + "sil_linearDampingCompensationCurrent", registry);
      sil_coggingCompensationMotorCurrent = new YoDouble(prefix + "sil_coggingCompensationMotorCurrent", registry);
      sil_dahlFrictionCompensationCurrent = new YoDouble(prefix + "sil_dahlFrictionCompensationCurrent", registry);
      sil_acceleratiojnIntegrationMotorFeedbackCurrent = new YoDouble(prefix + "sil_accelerationIntegrationMotorFeedbackCurrent", registry);
      sil_accelerationIntegrationMeasuredMotorPosition = new YoDouble(prefix + "sil_accelerationIntegrationMeasuredMotorPosition", registry);
      sil_accelerationIntegrationMeasuredMotorVelocity = new YoDouble(prefix + "sil_accelerationIntegrationMeasuredMotorVelocity", registry);

      // SIL Socket error and warning status signals
      inputEncoderWarningValue = new YoDouble(prefix + "sil_inputEncoderWarningValue", registry);
      inputEncoderErrorValue = new YoDouble(prefix + "sil_inputEncoderErrorValue", registry);
      inputEncoderState = new YoEnum<>(prefix + "InputEncoderState", registry, EncoderState.class);

      outputEncoderWarningValue = new YoDouble(prefix + "sil_outputEncoderWarningValue", registry);
      outputEncoderErrorValue = new YoDouble(prefix + "sil_outputEncoderErrorValue", registry);
      outputEncoderState = new YoEnum<>(prefix + "OutputEncoderState", registry, EncoderState.class);

      //actuals
      measuredMotorPosition = new YoDouble(prefix + "measuredMotorPosition", registry);
      measuredMotorVelocity = new YoDouble(prefix + "measuredMotorVelocity", registry);
      measuredMotorVelocityFD = new YoDouble(prefix + "measuredMotorVelocityFD", registry);
      useFDforMotorVelocity = new YoBoolean(prefix + "useFDforMotorVelocity", registry);
      useFDforMotorVelocity.set(false);

      filteredVelocityAlphaValue = new YoDouble(prefix + "filteredVelocityAlphaValue", registry);
      filteredVelocityAlphaValue.set(0.5);
      preFilteredMotorVelocity = new AlphaFilteredYoVariable(prefix + "preFilteredMotorVelocity", registry, filteredVelocityAlphaValue, measuredMotorVelocity);
      filteredMotorVelocity = new AlphaFilteredYoVariable(prefix + "filteredMotorVelocity", registry, filteredVelocityAlphaValue, preFilteredMotorVelocity);

      measuredMotorPositionInOutput = new YoDouble(prefix + "measuredMotorPositionInOutput", registry);
      measuredMotorVelocityInOutput = new YoDouble(prefix + "measuredMotorVelocityInOutput", registry);

      measuredOutputPosition = new YoDouble(prefix + "measuredOutputPosition", registry);
      measuredOutputVelocity = new YoDouble(prefix + "measuredOutputVelocity", registry);
      measuredOutputVelocityFD = new YoDouble(prefix + "measuredOutputVelocityFD", registry);
      useFDforOutputVelocity = new YoBoolean(prefix + "useFDforOutputVelocity", registry);
      useFDforOutputVelocity.set(false);

      filteredOutputVelocityAlphaValue = new YoDouble(prefix + "filteredOutputVelocityAlphaValue", registry);
      filteredOutputVelocityAlphaValue.set(0.5);
      preFilteredOutputVelocity = new AlphaFilteredYoVariable(prefix + "preFilteredOutputVelocity",
                                                              registry,
                                                              filteredOutputVelocityAlphaValue,
                                                              measuredOutputVelocity);
      filteredOutputVelocity = new AlphaFilteredYoVariable(prefix + "filteredOutputVelocity",
                                                           registry,
                                                           filteredOutputVelocityAlphaValue,
                                                           preFilteredOutputVelocity);

      measuredMotorCurrent = new YoDouble(prefix + "measuredMotorCurrent", registry);
      estimatedMotorTorque = new YoDouble(prefix + "estimatedMotorTorque", registry);
      estimatedOutputTorque = new YoDouble(prefix + "estimatedOutputTorque", registry);

      measuredAnalogInput2 = new YoDouble(prefix + "measuredAnalogInput2", registry);

      //actuals in raw units
      rawMeasuredMotorPosition = new YoInteger(prefix + "rawMeasuredMotorPosition", registry);
      rawMeasuredMotorVelocity = new YoDouble(prefix + "rawMeasuredMotorVelocity", registry);
      rawMeasuredOuputPosition = new YoDouble(prefix + "rawMeasuredOuputPosition", registry);
      rawMeasuredOutputVelocity = new YoDouble(prefix + "rawMeasuredOutputVelocity", registry);
      rawMeasuredMotorCurrent = new YoInteger(prefix + "rawMeasuredMotorCurrent", registry);

      //settings and op stuff
      requestedModeOfOperation = new YoEnum<>(prefix + "requestedModeOfOperation", registry, ElmoModeOfOperation.class);
      currentModeOfOperation = new YoEnum<>(prefix + "currentModeOfOperation", registry, ElmoModeOfOperation.class);
      enableDrive = new YoBoolean(prefix + "enableDrive", registry);
      clearFaults = new YoBoolean(prefix + "clearFaults", registry);
      clearFaults.set(CLEAR_FAULTS);
      stayDisabled = new YoBoolean(prefix + "stayDisabled", registry);
      stayDisabled.set(false);

      controlWord = new YoEnum<>(prefix + "controlWord", registry, DSP402Slave.ControlWord.class, true);
      statusWord = new YoEnum<>(prefix + "statusWord", registry, DSP402Slave.StatusWord.class);
      elmoStatusRegister = new YoLong(prefix + "elmoStatusRegister", registry);
      errorCode = new YoInteger(prefix + "elmoErrorCode", registry);
      elmoErrorString = new YoEnum<>(prefix + "elmoErrorString", "", registry, true, ElmoErrorCodes.EC);
      measuredBusVoltage = new YoDouble(prefix + "busVoltage", registry);

      //faults
      statusRegisterProcessor = new ElmoTwitterStatusRegisterProcessor(registry);
      DRIVE_FAULTED = new YoBoolean(prefix + "_DRIVE_FAULTED", registry);
      UNDER_VOLTAGE = new YoBoolean(prefix + "_UNDER_VOLTAGE", registry);
      OVER_VOLTAGE = new YoBoolean(prefix + "_OVER_VOLTAGE", registry);
      STO_DISABLED = new YoBoolean(prefix + "_STO_DISABLED", registry);
      CURRENT_SHORT = new YoBoolean(prefix + "_CURRENT_SHORT", registry);
      OVER_TEMPERATURE = new YoBoolean(prefix + "_OVER_TEMPERATURE", registry);
      MOTOR_ENABLED = new YoBoolean(prefix + "_MOTOR_ENABLED", registry);
      MOTOR_FAULT = new YoBoolean(prefix + "_MOTOR_FAULT", registry);
      CURRENT_LIMITED = new YoBoolean(prefix + "_CURRENT_LIMITED", registry);

      DRIVE_FAULTED.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            if (DRIVE_FAULTED.getBooleanValue())
            {
               MOTOR_FAULT.set(true);
               LogTools.error("Drive fault at " + name + " with error: " + elmoErrorString.getValue());
            }
         }
      });

      requestedModeOfOperation.set(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);

      parentRegistry.addChild(registry);
   }

   public void setMotorDirection(boolean isMotorDirectionReversed)
   {
      if (isMotorDirectionReversed)
         motorDirection.set(-1.0);
      else
         motorDirection.set(1.0);
   }

   public void setSILParameters(SILParameters parameters)
   {
      dahlFrictionForce.set(parameters.getDahlFrictionForceGain());
      dahlSlope.set(parameters.getDahlFrictionSlope());
      linearDampingCompensation.set(parameters.getLinearDampingCompensationGain());

      dahlOutputScalar.set(parameters.getDahlOutputScalar());
      linearDampingOutputScalar.set(parameters.getLinearDampingOutputScalar());
      coggingOutputScalar.set(parameters.getCoggingOutputScalar());

      // AccelerationIntegration Parameters
      accelerationIntegrationScalar.set(parameters.getAccelerationIntegrationScalar());

      accelerationIntegrationScalar.set(1.0);
   }

   @Override
   public void read()
   {
      // estimate the update dt. We're finite differencing the velocity, so making sure this estimate is accurate is really important. If we don't have a
      // previous update time, then we can fall back to the provided dt.
      if (previousTime.isNaN())
      {
         estimatedDt.set(dt);
         previousTime.set(time.getValue());
      }
      else
      {
         estimatedDt.set(time.getValue() - previousTime.getDoubleValue());
         previousTime.set(time.getValue());
      }

      /*
       * Get drive status
       */
      statusWord.set(platinumTwitter.getStatus());

      int rawElmoStatusRegisterValue = platinumTwitter.getElmoStatusRegister();
      elmoStatusRegister.set(rawElmoStatusRegisterValue);
      statusRegisterProcessor.processStatusRegisterBits(rawElmoStatusRegisterValue);

      controlWord.set(platinumTwitter.getCurrentControlword());
      errorCode.set(platinumTwitter.getErrorRegister());
      //TODO Implement fully
      //      elmoErrorString.set(errorCode.getIntegerValue());
      currentModeOfOperation.set(platinumTwitter.getModeOfOperation());

      DRIVE_FAULTED.set(platinumTwitter.isFaulted() || !platinumTwitter.isOperational());
      UNDER_VOLTAGE.set(platinumTwitter.isUnderVoltage());
      OVER_VOLTAGE.set(platinumTwitter.isOverVoltage());
      STO_DISABLED.set(platinumTwitter.isSTODisabled());
      CURRENT_SHORT.set(platinumTwitter.isCurrentShorted());
      OVER_TEMPERATURE.set(platinumTwitter.isOverTemperature());

      // Update SIL readable variables
      sil_dahlFrictionCompensationCurrent.set(platinumTwitter.getSILDahlFrictionCompensationCurrent());
      sil_linearDampingCompensationCurrent.set(platinumTwitter.getSILLinearDampingCompensationCurrent());
      sil_coggingCompensationMotorCurrent.set(platinumTwitter.getSILDesiredCoggingCompensationCurrent());
      sil_acceleratiojnIntegrationMotorFeedbackCurrent.set(platinumTwitter.getSILDesiredPDControlFeedbackCurrent());

      sil_accelerationIntegrationMeasuredMotorPosition.set(platinumTwitter.getSILDesiredFeedForwardCurrent());
      sil_accelerationIntegrationMeasuredMotorVelocity.set(platinumTwitter.getSILDesiredTotalCurrent());

      inputEncoderWarningValue.set(platinumTwitter.getSocket1Warning());
      inputEncoderErrorValue.set(platinumTwitter.getSocket1Error());
      outputEncoderWarningValue.set(platinumTwitter.getSocket2Warning());
      outputEncoderErrorValue.set(platinumTwitter.getSocket2Error());

      updateEncoderStates();

      measuredBusVoltage.set(platinumTwitter.getDCLinkVoltageMilliVolts() / 1000.0);

      /** Motor Space Encoders **/

      //raw motor position and velocity in counts & counts per sec
      rawMeasuredMotorPosition.set(platinumTwitter.getRawMotorEncoderPosition());
      rawMeasuredMotorVelocity.set(platinumTwitter.getRawMotorVelocity());

      // convert the motor encoder count measurement to the motor position in radians. Flip the sign here if the directionality is reversed 
      double currentMeasuredMotorPosition = motorDirection.getDoubleValue() * rawMeasuredMotorPosition.getIntegerValue() * motorEncoderCountsToMotorRadians;

      // get the motor position on the previous tick. This is used to finite difference the motor position to get the motor velocity.
      double previousMeasuredMotorPosition = measuredMotorPosition.getDoubleValue();

      // update the actual measured position of the motor
      measuredMotorPosition.set(currentMeasuredMotorPosition);

      // finite difference the measured motor velocity, looking at the previous encoder measurement.
      measuredMotorVelocityFD.set((currentMeasuredMotorPosition - previousMeasuredMotorPosition) / estimatedDt.getDoubleValue());

      // convert the motor velocity from counts per second in encoder space to output encoder radians per second.
      measuredMotorVelocity.set(
            motorDirection.getDoubleValue() * rawMeasuredMotorVelocity.getDoubleValue() * RAW_VELOCITY_TO_COUNTS_PER_SEC * motorEncoderCountsToMotorRadians);

      // perform low-pass filtering on the measured motor velocity signal.
      preFilteredMotorVelocity.update();

      // perform a second round of low-pass filtering on the filtered motor velocity signal.
      filteredMotorVelocity.update();

      //project motor measureds in output space
      measuredMotorPositionInOutput.set(measuredMotorPosition.getDoubleValue() / gearRatio.getDoubleValue());
      measuredMotorVelocityInOutput.set(measuredMotorVelocity.getValue() / gearRatio.getDoubleValue());

      /** Output Space Encoders **/

      //raw output position and velocity in counts & counts per sec
      rawMeasuredOuputPosition.set(platinumTwitter.getRawAuxiliaryPosition());
      rawMeasuredOutputVelocity.set(platinumTwitter.getRawAuxiliaryVelocity());

      // convert the output encoder count measurement to the output position in radians. Flip the sign here if the directionality is reversed
      double currentMeasuredOutputPosition = motorDirection.getDoubleValue() * rawMeasuredOuputPosition.getDoubleValue() * outputEncoderCountsToOutputRadians;

      // get the output joint position on the previous tick.  This is used to finite difference the output position to get the output velocity.
      double previousMeasuredOutputPosition = measuredOutputPosition.getDoubleValue();

      // update the actual measured position of the output
      measuredOutputPosition.set(currentMeasuredOutputPosition - zeroPositionOffset.getDoubleValue());

      // finite difference the measured velocity, looking at the previous encoder measurement.
      measuredOutputVelocityFD.set((currentMeasuredOutputPosition - previousMeasuredOutputPosition) / estimatedDt.getDoubleValue());

      measuredOutputVelocity.set(
            motorDirection.getDoubleValue() * rawMeasuredOutputVelocity.getDoubleValue() * RAW_VELOCITY_TO_COUNTS_PER_SEC * outputEncoderCountsToOutputRadians);

      // perform low-pass filtering on the measured output velocity signal.
      preFilteredOutputVelocity.update();
      // perform a second round of low-pass filtering on the filtered motor velocity signal.
      filteredOutputVelocity.update();

      /** Current and Torque **/

      // measured motor current
      rawMeasuredMotorCurrent.set(((int) motorDirection.getDoubleValue()) * platinumTwitter.getRawMeasuredCurrent());

      double measuredCurrentPercentage = rawMeasuredMotorCurrent.getIntegerValue() / CURRENT_SIGNAL_RANGE;
      // convert this percentage of the signal to the measured current.
      measuredMotorCurrent.set(measuredCurrentPercentage * platinumTwitter.getMaxDriveCurrentAmps());
      // convert the measured current to the torque.
      estimatedMotorTorque.set(measuredMotorCurrent.getDoubleValue() * kt.getDoubleValue());
      // convert the motor torque to joint torque
      estimatedOutputTorque.set(estimatedMotorTorque.getDoubleValue() * gearRatio.getDoubleValue());

      // get the maximum drive current that has been used, in milliamps.
      maxDriveCurrentMilliAmps.set(platinumTwitter.getMaxDriveCurrentMilliAmps());

      //read the voltage on analog input 2
      measuredAnalogInput2.set(platinumTwitter.getAnalogInput2());
   }

   private void updateEncoderStates()
   {

      // Updating Input Encoder State
      if (inputEncoderErrorValue.getDoubleValue() > 0.0)
      {
         inputEncoderState.set(EncoderState.ERROR);
      }
      else if (inputEncoderWarningValue.getDoubleValue() > 0.0)
      {
         inputEncoderState.set(EncoderState.WARNING);
      }
      else
      {
         inputEncoderState.set(EncoderState.NORMAL);
      }

      // Updating Output Encoder State
      if (outputEncoderErrorValue.getDoubleValue() > 0.0)
      {
         outputEncoderState.set(EncoderState.ERROR);
      }
      else if (outputEncoderWarningValue.getDoubleValue() > 0.0)
      {
         outputEncoderState.set(EncoderState.WARNING);
      }
      else
      {
         outputEncoderState.set(EncoderState.NORMAL);
      }
   }

   @Override
   public void write()
   {
      if (statusWord.getEnumValue() == StatusWord.FAULT)
      {
         if (clearFaults.getBooleanValue())
         {
            platinumTwitter.setEnableDrive(enableDrive.getBooleanValue());
         }
         else
         {
            platinumTwitter.setEnableDrive(false);
            return;
         }
      }

      if (statusWord.getEnumValue() == StatusWord.SWITCHONDISABLED && stayDisabled.getBooleanValue())
      {
         return;
      }

      // Set the drive input modes and enable status
      platinumTwitter.setModeOfOperation(requestedModeOfOperation.getEnumValue());
      platinumTwitter.setEnableDrive(enableDrive.getBooleanValue());

      //TODO: implement
      //      for (int i = 0; i < 6; i++)
      //      {
      //         amplifier.setDigitalOutput(i, digitalOutputs[i].getBooleanValue());
      //      }

      // Set the desired motor position in encoder counts. Flip the sign if the directionality is reversed so that it matches the motor axis.
      rawDesiredMotorPosition.set((int) (motorDirection.getDoubleValue() * (desiredMotorPosition.getDoubleValue() * outputRadiansToMotorEncoderCounts)));

      // Set the desired motor velocity in encoder counts per second. Flip the sign if the directionality is reversed so that it matches the motor axis.
      rawDesiredMotorVelocity.set((int) (motorDirection.getDoubleValue() * (desiredMotorVelocity.getDoubleValue() * outputRadiansToMotorEncoderCounts)));

      // Compute the total desired current for the drive. This is the summation of the feedforward motor current, the desired motor current, which is the main
      // setpoint of this drive and comes from the desired motor torque, and the velocity feedforward current. This is likely the same as the desired
      // feedforward motor current.
      desiredFeedForwardMotorCurrent.set(0.0); //TODO Figure out how to do this properly
      desiredMotorTorque.set(desiredOutputTorque.getDoubleValue() / gearRatio.getDoubleValue());
      desiredMotorCurrent.set(desiredMotorTorque.getDoubleValue() / kt.getDoubleValue());
      desiredMotorCurrentWithFF.set(desiredMotorCurrent.getDoubleValue() + desiredFeedForwardMotorCurrent.getDoubleValue());

      // Compute the desired motor current to an effort percentage, -1000 to 1000. This is a percentage of the maximum effort available from the drive. Flip the
      // sign of the current if the directionality is reversed so that it matches the motor axis.
      double desiredEffortPercentage = motorDirection.getDoubleValue() * EuclidCoreTools.clamp(desiredMotorCurrentWithFF.getDoubleValue()
                                                                                               / platinumTwitter.getMaxDriveCurrentAmps(), 1.0);
      rawDesiredMotorEffortPercentage.set((int) (CURRENT_SIGNAL_RANGE * desiredEffortPercentage));

      // Set the actual objectives for the drive. This includes the desired motor encoder counts, the desired motor encoder counts per second, and the
      // desired percentage of max effort, -1000 to 1000.
      platinumTwitter.setRawTargetPosition(rawDesiredMotorPosition.getIntegerValue());
      platinumTwitter.setRawTargetVelocity(rawDesiredMotorVelocity.getIntegerValue());
      platinumTwitter.setPercentageMaxEffort(rawDesiredMotorEffortPercentage.getIntegerValue());

      platinumTwitter.setR2NVMSaveFlag(saveR2ToNVM.getBooleanValue());

      // Set the desired SIL controller parameters to the amplifier.
      platinumTwitter.setDahlFrictionForce(dahlFrictionForce.getDoubleValue());
      platinumTwitter.setDahlSlope(dahlSlope.getDoubleValue());
      platinumTwitter.setLinearDampingCompensation(linearDampingCompensation.getDoubleValue());

      platinumTwitter.setCoggingOutputScalar(coggingOutputScalar.getDoubleValue());
      platinumTwitter.setDahlOutputScalar(dahlOutputScalar.getDoubleValue());
      platinumTwitter.setLinearDampingOutputScalar(linearDampingOutputScalar.getDoubleValue());

      // Set acceleration Integration Parameters
      platinumTwitter.setAccelerationIntegrationStiffness(accelerationIntegrationStiffness.getDoubleValue());
      platinumTwitter.setAccelerationIntegrationDamping(accelerationIntegrationDamping.getDoubleValue());
      platinumTwitter.setAccelerationIntegrationScalar(accelerationIntegrationScalar.getDoubleValue());

      platinumTwitter.setMotorDesiredPosition(motorDirection.getDoubleValue() * accelerationIntegrationDesiredMotorPosition.getDoubleValue());
      platinumTwitter.setMotorDesiredVelocity(motorDirection.getDoubleValue() * accelerationIntegrationDesiredMotorVelocity.getDoubleValue());

      platinumTwitter.setMaxMotorPositionError(accelerationIntegrationMaxPositionError.getDoubleValue());
      platinumTwitter.setMaxMotorVelocityError(accelerationIntegrationMaxVelocityError.getDoubleValue());

      platinumTwitter.doStateControl();
   }

   private void applyValueLimits(YoDouble variableToLimit, double lowerLimit, double upperLimit)
   {
      variableToLimit.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable yoVariable)
         {
            double value = variableToLimit.getDoubleValue();
            variableToLimit.set(MathTools.clamp(value, lowerLimit, upperLimit));
         }
      });
   }

   @Override
   public void enableDrive(boolean enable)
   {
      this.enableDrive.set(enable);
   }

   @Override
   public void setDesiredOutputTorque(double desiredTorque)
   {
      desiredOutputTorque.set(desiredTorque);
   }

   @Override
   public void setDesiredFeedForwardCurrent(double value)
   {
      desiredFeedForwardMotorCurrent.set(value);
   }

   @Override
   public void setDesiredMotorTorque(double desiredMotorTorque)
   {
      this.desiredMotorTorque.set(desiredMotorTorque);
   }

   @Override
   public void setDesiredMotorPosition(double motorPosition)
   {
      accelerationIntegrationDesiredMotorPosition.set(motorPosition);
   }

   @Override
   public void setDesiredMotorVelocity(double motorVelocity)
   {
      accelerationIntegrationDesiredMotorVelocity.set(motorVelocity);
   }

   @Override
   public boolean isMotorFaulted()
   {
      return MOTOR_FAULT.getBooleanValue();
   }

   public void setKt(double kt)
   {
      this.kt.set(kt);
   }

   public void reversePositiveMotorDirection()
   {
      motorDirection.set(-1);
   }

   public void setVelocityFilterAlpha(double velocityFilterAlpha)
   {
      filteredVelocityAlphaValue.set(velocityFilterAlpha);
      filteredOutputVelocityAlphaValue.set(velocityFilterAlpha);
   }

   public void enableCyclicSynchronousPosition()
   {
      requestedModeOfOperation.set(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_POSITION);
   }

   public void enableCyclicSynchronousVelocity()
   {
      requestedModeOfOperation.set(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_VELOCITY);
   }

   public void enableCyclicSynchronousTorque()
   {
      requestedModeOfOperation.set(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);
   }

   public double getJointPosition()
   {
      return getMeasuredOutputPosition();
   }

   public double getJointVelocity()
   {
      return getMeasuredOutputVelocity();
   }

   public boolean getDriveFaulted()
   {
      return DRIVE_FAULTED.getBooleanValue();
   }

   @Override
   public double getMeasuredOutputVelocity()
   {
      return measuredOutputVelocity.getValue();
   }

   @Override
   public double getMeasuredOutputTorque()
   {
      return estimatedOutputTorque.getDoubleValue();
   }

   @Override
   public double getMeasuredMotorTorque()
   {
      return estimatedMotorTorque.getDoubleValue();
   }

   @Override
   public double getGearRatio()
   {
      return gearRatio.getDoubleValue();
   }

   public double getKt()
   {
      return kt.getDoubleValue();
   }

   @Override
   public double getMeasuredMotorPosition()
   {
      return measuredMotorPosition.getDoubleValue();
   }

   @Override
   public double getMeasuredOutputPosition()
   {
      return measuredOutputPosition.getDoubleValue();
   }

   @Override
   public double getMeasuredMotorVelocity()
   {
      return measuredMotorVelocity.getDoubleValue();
   }

   @Override
   public double getFilteredMotorVelocity()
   {
      return filteredMotorVelocity.getDoubleValue();
   }

   @Override
   public double getFilteredOutputVelocity()
   {
      return filteredOutputVelocity.getDoubleValue();
   }

   @Override
   public State getEtherCATState()
   {
      return platinumTwitter.getState();
   }

   public CycloidActuatorParameters getActuatorParameters()
   {
      return actuatorParameters;
   }

   public void setAccelerationIntegrationDesiredInputPosition(double desiredPosition)
   {
      accelerationIntegrationDesiredMotorPosition.set(desiredPosition);
   }

   public void setAccelerationIntegrationDesiredInputVelocity(double desiredVelocity)
   {
      accelerationIntegrationDesiredMotorVelocity.set(desiredVelocity);
   }

   @Override
   public void setDesiredMotorStiffness(double desiredMotorStiffness)
   {
      accelerationIntegrationStiffness.set(desiredMotorStiffness);
   }

   @Override
   public void setDesiredMotorDamping(double desiredMotorDamping)
   {
      accelerationIntegrationDamping.set(desiredMotorDamping);
   }

   @Override
   public void setMaxPositionFeedbackError(double maxPositionFeedbackError)
   {
      accelerationIntegrationMaxPositionError.set(maxPositionFeedbackError);
   }

   @Override
   public void setMaxVelocityFeedbackError(double maxVelocityFeedbackError)
   {
      accelerationIntegrationMaxVelocityError.set(maxVelocityFeedbackError);
   }

   public double getStatorTemperature()
   {
      return convertAnalogInputToTemperatureInDegreeCelsius(this.measuredAnalogInput2.getValue());
   }

   public void setMaxAllowableStatorTemperature(int maxAllowableStatorTemperature)
   {
      this.maxAllowableStatorTemperature.set(maxAllowableStatorTemperature);
   }

   public int getMaxAllowableStatorTemperature()
   {
      return this.maxAllowableStatorTemperature.getValue();
   }

   public double convertAnalogInputToTemperatureInDegreeCelsius(double voltage)
   {
      return (TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[0] * Math.sqrt(voltage) + TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[1] * voltage
              + TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[2]);
   }

   public void setMaxRecommendedStatorTemperature(int maxRecommendedStatorTemperature)
   {
      this.maxRecommendedStatorTemperature.set(maxRecommendedStatorTemperature);
   }

   public int getMaxRecommendedStatorTemperature()
   {
      return this.maxRecommendedStatorTemperature.getValue();
   }

   public double getInputEncoderWarningSignal()
   {
      return inputEncoderWarningValue.getDoubleValue();
   }

   public double getInputEncoderErrorSignal()
   {
      return inputEncoderErrorValue.getDoubleValue();
   }

   public double getOutputEncoderWarningSignal()
   {
      return outputEncoderWarningValue.getDoubleValue();
   }

   public double getOutputEncoderErrorSignal()
   {
      return outputEncoderErrorValue.getDoubleValue();
   }

   public String getName()
   {
      return name;
   }

   public CycloidActuatorPackage getActuatorPackage()
   {
      return actuatorPackage;
   }

   public CycloidPlatinumTwitter getPlatinumTwitter()
   {
      return platinumTwitter;
   }

   public void setCompensation(boolean enable)
   {
      enableCompensation.set(enable);
   }
}