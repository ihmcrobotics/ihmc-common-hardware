package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.slaves.DSP402Slave.ControlWord;
import us.ihmc.etherCAT.slaves.DSP402Slave.StatusWord;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class YoEverestMotorController
{
   private static final boolean CLEAR_FAULTS = true;
   private static final int ENCODER_RESOLUTION = 17;
   private static final double ENCODER_CONVERSION = MathTools.pow(2.0, ENCODER_RESOLUTION);
   private static final double FULL_ROTATION = 2.0 * Math.PI;
   private final YoRegistry registry;
   private final EverestMotorController motorController;

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

   private final YoDouble motorPositionOffset;
   private final YoDouble actuatorPositionOffset;
   private final YoBoolean findOffset;

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

   private final String name;

   public YoEverestMotorController(String prefix, EverestMotorController motorController, YoRegistry parentRegistry)
   {
      this.motorController = motorController;
      name = prefix + "EverestMC";
      registry = new YoRegistry(name);

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

      parentRegistry.addChild(registry);

      torqueConstant.set(motorController.getTorqueConstant());
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
      torqueConstant.set(motorController.getTorqueConstant());
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

      motorController.doStateControl();
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


}
