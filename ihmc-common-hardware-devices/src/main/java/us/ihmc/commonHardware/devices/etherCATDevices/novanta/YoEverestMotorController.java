package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

import us.ihmc.commonHardware.devices.cycloids.TwitterEncoderStatusManager;
import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.slaves.DSP402Slave.ControlWord;
import us.ihmc.etherCAT.slaves.DSP402Slave.StatusWord;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.etherCAT.master.Slave.State;

public class YoEverestMotorController
{
   private static boolean maxConfig = false;
   private static final boolean CLEAR_FAULTS = true;
   private static final int ENCODER_RESOLUTION = 17;
   private static final double ENCODER_CONVERSION = MathTools.pow(2.0, ENCODER_RESOLUTION);
   private static final double FULL_ROTATION = 2.0 * Math.PI;
   private final YoRegistry registry;
   private final YoRegistry maxConfigRegistery;
   private final EverestMotorController motorController;

   private final YoBoolean DRIVE_FAULTED;
   private final YoBoolean UNDER_VOLTAGE;
   private final YoBoolean OVER_VOLTAGE;
   private final YoBoolean STO_DISABLED;
   private final YoBoolean CURRENT_SHORT;
   private final YoBoolean OVER_TEMPERATURE;
   private final YoBoolean MOTOR_FAULT;

   private final TwitterEncoderStatusManager inputEncoderStatusManager;
   private final TwitterEncoderStatusManager outputEncoderStatusManager;

   private final YoEnum<State> etherCATState;

   private final YoDouble errorPersistenceThreshold;

   // YoVariables to write to the drive
   private final YoEnum<ControlWord> requestedControlWord;
   private final YoEnum<EverestOperationModes> requestedOperationMode;
   private final YoDouble desiredMotorCurrent;
   private final YoDouble desiredMotorPosition;
   private final YoDouble desiredMotorVelocity;
   private final YoInteger rawDesiredMotorPosition;
   private final YoDouble rawDesiredMotorVelocity;

   //YoVariables to read from the drive
   private final YoInteger rawMeasuredMotorPosition;
   private final YoDouble rawMeasuredMotorVelocity;
   private final YoLong rawMeasuredActuatorPosition;
   private final YoDouble rawMeasuredActuatorVelocity;

   private final YoDouble measuredMotorPosition;
   private final YoDouble measuredMotorVelocity;
   private final YoDouble measuredActuatorPosition;
   private final YoDouble measuredActuatorVelocity;
   private final YoDouble measuredTorque;
   private final YoDouble measuredTemperature;
   private final YoDouble desiredTorque;
   private final YoDouble desiredMotorDamping;
   private final YoDouble desiredMotorStiffness;
   private final YoDouble maxTorque;

   private final YoDouble motorPositionOffset;
   private final YoDouble actuatorPositionOffset;
   private final YoBoolean findOffset;
   private final YoDouble offsetFromZero;


   private final YoDouble measuredMotorCurrent;
   private final YoDouble commandedMotorCurrent;
   private final YoDouble busVoltage;
   private final YoDouble motorTemperature;

   private final YoEnum<StatusWord> statusWord;
   private final YoInteger errorCode;
   private final YoEnum<EverestErrorCodes> errorMessage;
   private final YoEnum<EverestOperationModes> currentOperationMode;

   private final YoBoolean enableDrive;
   private final YoBoolean clearFaults;
   private final YoBoolean stayDisabled;

   private final YoDouble torqueConstant;
   private final YoDouble estimatedMotorTorque;

   private final YoDouble currentDirectSetPoint;
   private final YoDouble torqueSetPoint;
   private final YoDouble voltageQuadratureSetPoint;
   private final YoDouble voltageDirectSetPoint;
   private final YoDouble currentASetPoint;
   private final YoDouble currentBSetPoint;
   private final YoInteger targetTorque;
   private final YoInteger torqueOffset;
   private final YoDouble digitalOutputSetValue;
   private final YoDouble analogOutputSetValue;
   //private final YoDouble auxiliaryFeedbackValue;
   private final YoDouble currentDirectValue;
   private final YoDouble motorTempValue;
   private final YoDouble powerStageTemp1Value;
   private final YoDouble followingError;
   private final YoDouble maxTemp;
   private final YoDouble recommendedTemp;
   private final YoBoolean dynamicBrakingEnabled;

   private final YoBoolean enableCompensationCurrents;
   //private final YoBoolean motorFaulted;

   private final YoDouble outputPositionBreakFrequency;
   private final YoDouble outputVelocityBreakFrequency;
   private final YoDouble motorVelocityBreakFrequency;
   private final YoDouble motorPositionBreakFrequency;
   private final YoDouble maxPositionFeedbackError;
   private final YoDouble maxVelocityFeedbackError;

   private final String name;

   public YoEverestMotorController(String prefix, EverestMotorController motorController, YoRegistry parentRegistry, boolean maxConfig)
   {
      this.maxConfig = maxConfig;
      this.motorController = motorController;
      name = prefix + "EverestMC";
      registry = new YoRegistry(name);
      maxConfigRegistery = new YoRegistry("MaxConfigRegistery");

      requestedControlWord = new YoEnum<>(prefix + "RequestedControlWord", registry, ControlWord.class, true);
      requestedOperationMode = new YoEnum<>(prefix + "RequestedOperationMode", registry, EverestOperationModes.class);
      requestedOperationMode.set(EverestOperationModes.CURRENT);
      desiredMotorCurrent = new YoDouble(prefix + "DesiredMotorCurrent", registry);
      desiredMotorPosition = new YoDouble(prefix + "DesiredMotorPosition_rad", registry);
      desiredMotorVelocity = new YoDouble(prefix + "DesiredMotorVelocity_rad_s", registry);
      rawDesiredMotorPosition = new YoInteger(prefix + "DesiredMotorPosition_bits", registry);
      rawDesiredMotorVelocity = new YoDouble(prefix + "DesiredMotorVelocity_rev_s", registry);

      rawMeasuredMotorPosition = new YoInteger(prefix + "MotorPosition_bits", registry);
      rawMeasuredMotorVelocity = new YoDouble(prefix + "MotorVelocity_rev_s", registry);
      rawMeasuredActuatorPosition = new YoLong(prefix + "ActuatorPosition_bits", registry);
      rawMeasuredActuatorVelocity = new YoDouble(prefix + "ActuatorVelocity_rev_s", registry);

      desiredTorque = new YoDouble(prefix + "DesiredTorque", registry);
      desiredMotorDamping = new YoDouble(prefix + "DesiredMotorDamping", registry);
      desiredMotorStiffness = new YoDouble(prefix + "DesiredMotorTorque", registry);
      maxTorque = new YoDouble(prefix + "MaxTorque", registry);
      outputPositionBreakFrequency = new YoDouble(prefix + "OutputPositionBreakFrequency", registry);
      outputVelocityBreakFrequency = new YoDouble(prefix + "OutputVelocityBreakFrequency", registry);
      motorPositionBreakFrequency = new YoDouble(prefix + "MotorPositionBreakFrequency", registry);
      motorVelocityBreakFrequency = new YoDouble(prefix + "MotorVelocityBreakFrequency", registry);
      maxPositionFeedbackError = new YoDouble(prefix + "MaxPositionFeedbackError", registry);
      maxVelocityFeedbackError = new YoDouble(prefix + "MaxVelocityFeedbackError", registry);
      offsetFromZero = new YoDouble(prefix + "OffsetFromZero", registry);

      measuredMotorPosition = new YoDouble(prefix + "MotorPosition_rad", registry);
      measuredMotorVelocity = new YoDouble(prefix + "MotorVelocity_rad_s", registry);
      measuredActuatorPosition = new YoDouble(prefix + "ActuatorPosition_rad", registry);
      measuredActuatorVelocity = new YoDouble(prefix + "ActuatorVelocity_rad_s", registry);
      measuredTorque = new YoDouble(prefix + "Torque", registry);
      measuredTemperature = new YoDouble(prefix + "Temperature", registry);

      torqueConstant = new YoDouble(prefix + "TorqueConstant", registry);
      estimatedMotorTorque = new YoDouble(prefix + "EstimatedMotorTorque", registry);

      motorPositionOffset = new YoDouble(prefix + "MotorPositionOffset", registry);
      actuatorPositionOffset = new YoDouble(prefix + "ActuatorPositionOffset", registry);
      findOffset = new YoBoolean(prefix + "FindPositionOffset", registry);
      enableCompensationCurrents = new YoBoolean(prefix + "EnableCompensationCurrents", registry);
      //motorFaulted = new YoBoolean(prefix + "MotorFaulted", registry);

      DRIVE_FAULTED = new YoBoolean(prefix + "_DRIVE_FAULTED", registry);
      UNDER_VOLTAGE = new YoBoolean(prefix + "_UNDER_VOLTAGE", registry);
      OVER_VOLTAGE =  new YoBoolean(prefix + "_OVER_VOLTAGE", registry);
      STO_DISABLED = new YoBoolean(prefix + "_STO_DISABLED", registry);
      CURRENT_SHORT = new YoBoolean(prefix + "_CURRENT_SHORT", registry);
      OVER_TEMPERATURE = new YoBoolean(prefix + "_OVER_TEMPERATURE", registry);
      MOTOR_FAULT = new YoBoolean(prefix + "_MOTOR_FAULT", registry);

      etherCATState = new YoEnum<>(prefix + "_EC_State", registry, State.class);

      errorPersistenceThreshold = new YoDouble(prefix + "encoderErrorPersistenceThreshold", registry);
      errorPersistenceThreshold.set(1.0);

      inputEncoderStatusManager = new TwitterEncoderStatusManager(prefix + "_input", errorPersistenceThreshold, registry);
      outputEncoderStatusManager = new TwitterEncoderStatusManager(prefix + "_output", errorPersistenceThreshold, registry);

      findOffset.addListener(s -> {
         if (findOffset.getBooleanValue())
         {
            motorPositionOffset.set(measuredMotorPosition.getValue());
            actuatorPositionOffset.set(measuredActuatorPosition.getValue());
         }
      });

      measuredMotorCurrent = new YoDouble(prefix + "MeasuredMotorCurrent", registry);
      commandedMotorCurrent = new YoDouble(prefix + "commandedMotorCurrent", registry);
      busVoltage = new YoDouble(prefix + "BusVoltage", registry);
      motorTemperature = new YoDouble(prefix + "MotorTemperature", registry);

      statusWord = new YoEnum<>(prefix + "StatusWord", registry, StatusWord.class);
      errorCode = new YoInteger(prefix + "ErrorCode", registry);
      errorMessage = new YoEnum<>(prefix + "ErrorMessage", registry, EverestErrorCodes.class);
      currentOperationMode = new YoEnum<>(prefix + "CurrentOperationMode", registry, EverestOperationModes.class);

      enableDrive = new YoBoolean(prefix + "EnableDrive", registry);
      clearFaults = new YoBoolean(prefix + "ClearFaults", registry);
      clearFaults.set(CLEAR_FAULTS);
      stayDisabled = new YoBoolean(prefix + "stayDisabled", registry);
      stayDisabled.set(false);

      maxTemp = new YoDouble(prefix + "MaxTemp", registry);
      maxTemp.set(0);
      recommendedTemp = new YoDouble(prefix + "RecommendedTemp", registry);
      recommendedTemp.set(0);
      dynamicBrakingEnabled = new YoBoolean(prefix + "DynamicBreakingEnabled", registry);
      dynamicBrakingEnabled.set(false);

      parentRegistry.addChild(registry);
      if(maxConfig){
         parentRegistry.addChild(maxConfigRegistery);
      }

      torqueConstant.set(0.1579);
//      motorController.setTorqueConstant((float) torqueConstant.getValue());

//      torqueConstant.addListener(s -> motorController.setTorqueConstant((float) torqueConstant.getValue()));


      currentDirectSetPoint = new YoDouble(name + "CurrentDirectSetPoint", maxConfigRegistery);
      torqueSetPoint = new YoDouble(name + "TorqueSetPoint", maxConfigRegistery);
      voltageQuadratureSetPoint = new YoDouble(name + "VoltageQuadratureSetPoint", maxConfigRegistery);
      voltageDirectSetPoint = new YoDouble(name + "VoltageDirectSetPoint", maxConfigRegistery);
      currentASetPoint = new YoDouble(name + "CurrentASetPoint", maxConfigRegistery);
      currentBSetPoint = new YoDouble(name + "CurrentBSetPoint", maxConfigRegistery);
      targetTorque = new YoInteger(name + "TargetTorque", maxConfigRegistery);
      torqueOffset = new YoInteger(name + "TorqueOffset", maxConfigRegistery);
      digitalOutputSetValue = new YoDouble(name + "DigitalOutputSetValue", maxConfigRegistery);
      analogOutputSetValue = new YoDouble(name + "AnalogOutputSetValue", maxConfigRegistery);
      //auxiliaryFeedbackValue = new YoDouble(name + "AuxiliaryFeedbackSetValue", registry);
      currentDirectValue = new YoDouble(name + "CurrentDirectValue", maxConfigRegistery);
      motorTempValue = new YoDouble(name + "MotorTempValue", maxConfigRegistery);
      powerStageTemp1Value = new YoDouble(name + "PowerStageTemp1Value", maxConfigRegistery);
      followingError = new YoDouble(name + "FollowingError", maxConfigRegistery);

   }

   public void read()
   {
      requestedControlWord.set(motorController.getCurrentControlword());
      statusWord.set(motorController.getStatus());
      currentOperationMode.set(EverestOperationModes.fromByte(motorController.getCurrentOperationMode()));
      errorCode.set(motorController.getErrorCode());
      errorMessage.set(EverestErrorCodes.fromCode(errorCode.getValue()));

      rawMeasuredMotorPosition.set(motorController.getMotorPosition());
      rawMeasuredMotorVelocity.set(motorController.getMotorVelocity());
      rawMeasuredActuatorPosition.set(motorController.getActuatorPosition());
      rawMeasuredActuatorVelocity.set(motorController.getActuatorVelocity());

      measuredMotorPosition.set(rawMeasuredMotorPosition.getValue() / ENCODER_CONVERSION * FULL_ROTATION - motorPositionOffset.getValue());
      measuredMotorVelocity.set(rawMeasuredMotorVelocity.getValue() * FULL_ROTATION);
      measuredActuatorPosition.set(rawMeasuredActuatorPosition.getValue() / ENCODER_CONVERSION * FULL_ROTATION - actuatorPositionOffset.getValue());
      measuredActuatorVelocity.set(rawMeasuredActuatorVelocity.getValue() * FULL_ROTATION);
      measuredTorque.set(motorController.getActualTorque());
      measuredTemperature.set(motorController.getTemperature());

      measuredMotorCurrent.set(motorController.getQuadratureCurrent());
      commandedMotorCurrent.set(motorController.getCommandedCurrent());
      busVoltage.set(motorController.getBusVoltage());
      motorTemperature.set(motorController.getMotorTemperature());

      estimatedMotorTorque.set(measuredMotorCurrent.getValue()*torqueConstant.getValue());
     // torqueConstant.set(motorController.getTorqueConstant());
      if(maxConfig){
         currentDirectSetPoint.set(motorController.getCurrentDirectSetPoint());
         torqueSetPoint.set(motorController.getTorqueSetPoint());
         voltageQuadratureSetPoint.set(motorController.getVoltageQuadratureSetPoint());
         voltageDirectSetPoint.set(motorController.getVoltageDirectSetPoint());
         currentASetPoint.set(motorController.getCurrentASetPoint());
         currentBSetPoint.set(motorController.getCurrentBSetPoint());
         targetTorque.set(motorController.getTargetTorque());
         torqueOffset.set(motorController.getTorqueOffset());
         digitalOutputSetValue.set(motorController.getDigitalOutputSetValue());
         analogOutputSetValue.set(motorController.getPositionLoopKd());
         //auxiliaryFeedbackValue.set(motorController.getAuxiliaryFeedbackValue());
         currentDirectValue.set(motorController.getCurrentDirectValue());
         motorTempValue.set(motorController.getMotorTemperature());
         powerStageTemp1Value.set(motorController.getPowerStageTemp1Value());
         followingError.set(motorController.getFollowingError());
      }
   }

   public void write()
   {
      if (statusWord.getEnumValue() == StatusWord.FAULT)
      {
         if (clearFaults.getBooleanValue())
         {
            motorController.setEnableDrive(enableDrive.getBooleanValue());
         }
         else
         {
            motorController.setEnableDrive(false);
            enableDrive.set(false);
            return;
         }
      }

      if (statusWord.getEnumValue() == StatusWord.SWITCHONDISABLED && stayDisabled.getBooleanValue())
      {
         return;
      }

      motorController.setOperationMode(requestedOperationMode.getValue().getValue());
      motorController.setEnableDrive(enableDrive.getBooleanValue());

      rawDesiredMotorPosition.set((int) ((desiredMotorPosition.getValue() + motorPositionOffset.getValue()) / FULL_ROTATION * ENCODER_RESOLUTION));
      rawDesiredMotorVelocity.set(desiredMotorVelocity.getValue() / FULL_ROTATION);

      motorController.setDesiredCurrent(desiredMotorCurrent.getValue());
      motorController.setDesiredPosition(rawDesiredMotorPosition.getValue());
      motorController.setDesiredVelocity(rawDesiredMotorVelocity.getValue());
//      motorController.setTorqueConstant((float) torqueConstant.getValue());
      motorController.doStateControl();
   }

   private void initializeFaultDiagnostics()
   {
      DRIVE_FAULTED.addListener(source ->
      {
         if (DRIVE_FAULTED.getBooleanValue() && isDriveEnabled())
         {
            LogTools.error(getName() + " just faulted");
            MOTOR_FAULT.set(true);
         }
      });

      etherCATState.addListener(source ->
      {
         if (etherCATState.getEnumValue() == Slave.State.OFFLINE)
            LogTools.error(getName() + " just went OFFLINE");
         else if (etherCATState.getEnumValue() == Slave.State.SAFE_OPERR)
            LogTools.error(getName() + " just went to SAFE_OPERR");
      });

      UNDER_VOLTAGE.addListener(s ->
      {
         if (UNDER_VOLTAGE.getBooleanValue() && !isMotorFaulted() && isDriveEnabled())
            LogTools.error(getName() + " faulted due to under voltage, bus voltage dropped to " + busVoltage.getValue());
      });

      OVER_VOLTAGE.addListener(s ->
      {
         if (OVER_VOLTAGE.getBooleanValue() && !isMotorFaulted() && isDriveEnabled())
            LogTools.error(getName() + " faulted due to over voltage, bus voltage rose to " + busVoltage.getValue());
      });

      CURRENT_SHORT.addListener(s ->
      {
         if (CURRENT_SHORT.getBooleanValue() && !isMotorFaulted())
            LogTools.error(getName() + " faulted due to current short");
      });

      OVER_TEMPERATURE.addListener(s ->
      {
         if (OVER_TEMPERATURE.getBooleanValue() && !isMotorFaulted())
            LogTools.error(getName() + " faulted due to drive overheating at " + measuredTemperature.getValue());
      });

      STO_DISABLED.addListener(s ->
      {
         if (STO_DISABLED.getBooleanValue() && !DRIVE_FAULTED.getBooleanValue())
            LogTools.error(getName() + " faulted due to STO being disabled");
      });
   }

   public void clearFaults()
   {
      clearFaults.set(true);
      MOTOR_FAULT.set(false);
      DRIVE_FAULTED.set(false);
      UNDER_VOLTAGE.set(false);
      OVER_VOLTAGE.set(false);
      STO_DISABLED.set(false);
      CURRENT_SHORT.set(false);
      OVER_TEMPERATURE.set(false);
      inputEncoderStatusManager.clearErrors();
      outputEncoderStatusManager.clearErrors();
   }

   public void setRequestedOperationMode(EverestOperationModes operationMode)
   {
      this.requestedOperationMode.set(operationMode);
   }

   public void setDesiredMotorPosition(double desiredMotorPosition)
   {
      this.desiredMotorPosition.set(desiredMotorPosition);
   }

   public void setDesiredMotorCurrent(double desiredMotorCurrent)
   {
      this.desiredMotorCurrent.set(desiredMotorCurrent);
   }

   public void setDesiredMotorVelocity(double desiredMotorVelocity)
   {
      this.desiredMotorVelocity.set(desiredMotorVelocity);
   }

   public double getTorqueConstant()
   {
      return torqueConstant.getDoubleValue();
   }

   public void setEnableDrive(boolean enableDrive)
   {
      this.enableDrive.set(enableDrive);
   }

   public boolean isDriveEnabled()
   {
      return this.enableDrive.getBooleanValue();
   }

   public double getMeasuredMotorCurrent()
   {
      return measuredMotorCurrent.getValue();
   }

   public double getFilteredOutputPosition(){
      return this.measuredMotorPosition.getValue();
   }

   public double getMeasuredOutputPosition(){
      return this.rawMeasuredMotorPosition.getValue();
   }

   public double getFilteredOutputVelocity(){
      return this.measuredMotorVelocity.getValue();
   }

   public double getMeasuredOutputVelocity(){
      return this.rawMeasuredMotorVelocity.getValue();
   }

   public int getGearRatio(){
      return 19;
   }

   public double getDesiredMotorPosition(){
      return desiredMotorPosition.getValue();
   }

   public double getMeasuredTorque(){
      return measuredTorque.getDoubleValue();
   }

   public double getMotorTemperature(){
      return this.motorTemperature.getValue();
   }

   public double getMaxMotorTemperature(){
      return this.maxTemp.getValue();
   }

   public double getRecommendedMotorTemperature(){
      return this.recommendedTemp.getValue();
   }

   public boolean isDynamicBrakingEnabled(){
      return this.dynamicBrakingEnabled.getBooleanValue();
   }

   public void setDesiredMotorTorque(double torque)
   {
      this.desiredTorque.set(torque);
   }

   public double getDesiredMotorTorque(){
      return this.desiredTorque.getValue();
   }

   public void setDesiredMotorDamping(double damping)
   {
      this.desiredMotorDamping.set(damping);
   }

   public double getDesiredMotorDamping(){
      return this.desiredMotorDamping.getValue();
   }

   public void setDesiredMotorStiffness(double stiffness){
      this.desiredMotorStiffness.set(stiffness);
   }

   public double getDesiredMotorStiffness(){
      return this.desiredMotorStiffness.getValue();
   }

   public boolean isMotorFaulted(){
      return this.MOTOR_FAULT.getBooleanValue();
   }

   public void setEnableCompensationCurrents(boolean enableCompensationCurrents){
      this.enableCompensationCurrents.set(enableCompensationCurrents);
   }

   public boolean isEnableCompensationCurrents(){
      return this.enableCompensationCurrents.getBooleanValue();
   }

   public double getMaxActuatorTorque(){
      return this.maxTorque.getValue();
   }

   public void setOutputPositionBreakFrequency(double freq){
      this.outputPositionBreakFrequency.set(freq);
   }

   public double getOutputPositionBreakFrequency(){
      return this.outputPositionBreakFrequency.getValue();
   }

   public void setOutputVelocityBreakFrequency(double freq){
      this.outputVelocityBreakFrequency.set(freq);
   }

   public double getOutputVelocityBreakFrequency(){
      return this.outputVelocityBreakFrequency.getValue();
   }

   public void setMotorPositionBreakFrequency(double freq){
      this.motorPositionBreakFrequency.set(freq);
   }

   public double getMotorPositionBreakFrequency(){
      return this.motorPositionBreakFrequency.getValue();
   }

   public void setMotorVelocityBreakFrequency(double freq){
      this.motorVelocityBreakFrequency.set(freq);
   }

   public double getMotorVelocityBreakFrequency(){
      return this.motorVelocityBreakFrequency.getValue();
   }

   public void setMaxPositionFeedbackError(double error){
      this.maxPositionFeedbackError.set(error);
   }

   public double getMaxPositionFeedbackError(){
      return this.maxPositionFeedbackError.getValue();
   }

   public void setMaxVelocityFeedbackError(double error){
      this.maxVelocityFeedbackError.set(error);
   }

   public double getMaxVelocityFeedbackError(){
      return this.maxVelocityFeedbackError.getValue();
   }

   public void zeroEncodersWithOffset(double offset){
      this.offsetFromZero.set(offset);
      this.findOffset.set(true);
   }

   public String getName()
   {
      return name;
   }

}
