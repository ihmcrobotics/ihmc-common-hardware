package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.ElmoTwitterStatusRegisterProcessor;
import us.ihmc.commonHardware.devices.etherCATDevices.elmo.YoGenericTwitter;
import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.etherCAT.slaves.DSP402Slave;
import us.ihmc.etherCAT.slaves.DSP402Slave.StatusWord;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlCycloidParameterLoader;
import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlCycloidParameters;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
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

   private static final double DEFAULT_MOTOR_POSITION_BREAK_FREQUENCY = 100.0;
   private static final double DEFAULT_OUTPUT_POSITION_BREAK_FREQUENCY = 10000.0;
   private static final double DEFAULT_MOTOR_VELOCITY_BREAK_FREQUENCY = 100.0;
   private static final double DEFAULT_OUTPUT_VELOCITY_BREAK_FREQUENCY = 10000.0;

   private static final double DEFAULT_SOFTWARE_BASED_OVER_FAULT_THRESHOLD = 57.0;
   private static final double DEFAULT_SOFTWARE_BASED_UNDER_FAULT_THRESHOLD = Double.NEGATIVE_INFINITY;

   private final double dt;
   private final String name;
   private final YoRegistry registry;

   private final DoubleProvider time;
   private final YoDouble silTime;
   private final YoDouble silDT;
   private final YoDouble previousTime;
   private final YoDouble estimatedDt;

   private final YoBoolean useOutputVelocityFromMotor;
   private final YoBoolean useOutputPositionFromMotor;

   private final CycloidPlatinumTwitter platinumTwitter;

   // conversion variables
   private final double motorEncoderCountsToMotorRadians;
   private final double motorEncoderCountsToOutputRadians;
   private final double motorRadiansToMotorEncoderCounts;

   private final double outputEncoderCountsToOutputRadians;
   private final double outputRadiansToMotorEncoderCounts;

   // command variables
   protected final YoDouble desiredMotorPositionForImpedanceControl;
   protected final YoDouble desiredMotorVelocityForImpedanceControl;
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
   protected final YoEnum<ElmoModeOfOperation> previousModeOfOperation;

   // measured variables
   private final YoDouble rawMotorPositionFromTwitters;
   private final YoDouble rawMotorVelocityFromTwitters;
   private final YoDouble measuredMotorPosition;
   private final YoDouble measuredMotorVelocity;
   private final YoDouble filteredMotorPosition;
   private final YoDouble filteredMotorVelocity;
   private final YoDouble measuredMotorVelocityFD;
   private final YoBoolean useFDforMotorVelocity;

   private final YoDouble measuredOutputPositionFromMotor;
   private final YoDouble measuredOutputVelocityFromMotor;
   private final YoDouble filteredOutputPositionFromMotor;
   private final YoDouble filteredOutputVelocityFromMotor;

   private final YoDouble measuredOutputPosition;
   private final YoDouble measuredOutputVelocity;
   private final YoDouble filteredOutputPosition;
   private final YoDouble filteredOutputVelocity;
   private final YoDouble measuredOutputVelocityFD;
   private final YoBoolean useFDforOutputVelocity;

   private final YoLong maxDriveCurrentMilliAmps;
   private final YoDouble measuredMotorCurrent;
   private final YoDouble estimatedMotorTorque;
   private final YoDouble estimatedOutputTorque;

   private final YoInteger rawMeasuredMotorCurrent;

   private final YoEnum<DSP402Slave.StatusWord> statusWord;
   private final YoLong elmoStatusRegister;
   private final YoInteger errorCode;
   //   private final YoEnum<?> elmoErrorString;
   private final YoDouble measuredBusVoltage;
   private final YoDouble measuredAnalogInput2InADCCounts;
   private final YoDouble measuredAnalogInput1InADCCounts;
   private final YoDouble measuredAnalogInput2InVolts;
   private final YoDouble measuredAnalogInput1InVolts;
   private final YoBoolean outputEncoderInverted;

   private final YoDouble statorTemp;

   // RTD 1000 temperature sensor function coefficients, these convert from volts to degrees celsius
   // These were found via thermal analysis performed by Liam Gluck during his summer 2025 internship
   private static final double[] TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS = new double[] {0, 334.0, -516.0};
   private static final boolean USE_ANALOG_1_FOR_STATOR_TEMP = true;

   // These convert from ADC counts to volts
   // These numbers came from the Nadia robot, which is why its in the name
   private static final double ANALOG_INPUT_1_CONVERSION_CONSTANT_FROM_NADIA = 0.0003729982709046532;
   private static final double ANALOG_INPUT_2_CONVERSION_CONSTANT_FROM_NADIA = -0.005940005648881197;

   private final YoBoolean useAnalog1ForStatorTemp;

   private final YoEnum<State> etherCATState;

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

   private final YoDouble dahlFrictionForce; // Dahl Friction Force
   private final YoDouble dahlSlope; // Dahl Slope (offset by + 1.0)
   private final YoDouble linearDampingCompensationGain; // Linear Damping Compensation
   private final YoDouble coggingOutputScalar; // Cogging Output Scalar

   // SIL Acceleration Integration Variables
   private final YoDouble impedanceControlStiffness;
   private final YoDouble impedanceControlDamping;
   private final YoDouble impedanceControlMaxPositionError;
   private final YoDouble impedanceControlMaxVelocityError;

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

   private final CycloidPhysicalParameters physicalParameters;
   private final CycloidSILParameters silParameters;

   // Tunable thresholds for under and over volt protection
   private final YoDouble softwareBasedOverVoltThreshold;
   private final YoDouble softwareBasedUnderVoltThreshold;

   // SIL Debuggging variables
   private final YoDouble sil_linearDampingCompensationCurrent;
   private final YoDouble sil_coggingCompensationMotorCurrent;
   private final YoDouble sil_dahlFrictionCompensationCurrent;
   private final YoDouble sil_impedanceControlMotorFeedbackCurrent;
   private final YoDouble sil_feedForwardCurrent;
   private final YoDouble sil_totalDesiredCurrent;

   // SIL Socket warning and error status variables
   private final YoDouble inputEncoderWarningValue;
   private final YoDouble inputEncoderErrorValue;
   private final YoDouble outputEncoderWarningValue;
   private final YoDouble outputEncoderErrorValue;

   private YoEnum<EncoderState> inputEncoderState;
   private YoEnum<EncoderState> outputEncoderState;

   private final String actuatorPackage;

   private final YoBoolean reverseMotorDirection;
   private final SimpleMovingAverageFilteredYoVariable averageOutputPosition;
   private final YoDouble outputPositionOffset;
   private final SimpleMovingAverageFilteredYoVariable averageMotorPosition;
   private final YoDouble motorPositionOffset;
   private final YoDouble encoderDifferenceAtOutput;
   private final YoBoolean zeroEncoders;
   private final int outputCountsPerRevolution;
   private final int inputCountsPerRevolution;
   private boolean firstRead = true;

   private final YoDouble driveTemperature;

   private final YoDouble motorPositionBreakFrequency;
   private final YoDouble outputPositionBreakFrequency;
   private final YoDouble motorVelocityBreakFrequency;
   private final YoDouble outputVelocityBreakFrequency;

   private final YoBoolean dynamicBrakingEnabled;

   private final YoBoolean checkEncoderOffsets;
   private double offsetFromZero;

   private enum EncoderState
   {
      NORMAL, WARNING, ERROR
   }

   public YoCycloidPlatinumTwitter(String prefix,
                                   CycloidPlatinumTwitter twitter,
                                   DoubleProvider time,
                                   String actuatorPackage,
                                   boolean isMotorDirectionReversed,
                                   int inputOffset,
                                   int outputOffset,
                                   double dt,
                                   YoRegistry parentRegistry)
   {
      this(prefix, twitter, time, null, actuatorPackage, isMotorDirectionReversed, inputOffset, outputOffset, dt, parentRegistry);
   }

   public YoCycloidPlatinumTwitter(String prefix,
                                   CycloidPlatinumTwitter twitter,
                                   DoubleProvider time,
                                   String actuatorDirectory,
                                   String actuatorPackage,
                                   boolean isMotorDirectionReversed,
                                   double motorOffset,
                                   double outputOffset,
                                   double dt,
                                   YoRegistry parentRegistry)

   {
      this(prefix, twitter, time, actuatorDirectory, actuatorPackage, isMotorDirectionReversed, motorOffset, outputOffset, dt, false, false, parentRegistry);
   }

   /**
    * Construct the vovariable wrapper of the platinum twitter connected to the cycloid
    *
    * @param prefix                    Prefix to be applied to yovariable names
    * @param twitter                   Twitter to be wrapped
    * @param time                      Controller time in seconds
    * @param actuatorDirectory         Directory where the xml for the cycloid parameters lives
    * @param actuatorPackage           Name of the actuator package
    * @param isMotorDirectionReversed  Decides if the signals from the motor are inverted or not relative to robot orientation
    * @param motorOffset               Offset of the input encoder in bits
    * @param outputOffset              Offset of the output encoder in bits
    * @param dt                        Controller timestep
    * @param enableCompensationAtStart Decide if SIL compensation currents are initially enabled or not
    * @param parentRegistry            Parent {code YoRegistry} of the twitter
    */
   public YoCycloidPlatinumTwitter(String prefix,
                                   CycloidPlatinumTwitter twitter,
                                   DoubleProvider time,
                                   String actuatorDirectory,
                                   String actuatorPackage,
                                   boolean isMotorDirectionReversed,
                                   double motorOffset,
                                   double outputOffset,
                                   double dt,
                                   boolean dynamicBrakingEnabled,
                                   boolean enableCompensationAtStart,
                                   YoRegistry parentRegistry)
   {
      this.time = time;
      this.dt = dt;
      this.platinumTwitter = twitter;
      this.actuatorPackage = actuatorPackage;
      name = prefix + getClass().getSimpleName();
      registry = new YoRegistry(name);
      XmlCycloidParameters cycloidParameters;
      if (actuatorDirectory != null)
         cycloidParameters = XmlCycloidParameterLoader.getCycloidParametersFromActuatorPackageName(actuatorDirectory, actuatorPackage);
      else
         cycloidParameters = XmlCycloidParameterLoader.getCycloidParametersFromActuatorPackageName(actuatorPackage);

      this.physicalParameters = new CycloidPhysicalParameters(cycloidParameters.getPhysicalParameters()); //CycloidPhysicalParameters.createCycloidParameters(actuatorPackage);
      this.silParameters = new CycloidSILParameters(cycloidParameters.getSilParameters()); //CycloidSILParameters.createParameters(actuatorPackage);
      this.dynamicBrakingEnabled = new YoBoolean(name + "DynamicBrakingIsEnabled", registry);
      this.dynamicBrakingEnabled.set(dynamicBrakingEnabled);

      this.reverseMotorDirection = new YoBoolean(name + "MotorDirectionIsReversed", registry);
      this.reverseMotorDirection.set(isMotorDirectionReversed);
      motorDirection = new YoDouble(prefix + "motorDirection", registry);
      setMotorDirection(isMotorDirectionReversed);

      encoderDifferenceAtOutput = new YoDouble(name + "EncoderDifferenceAtOutput", registry);

      zeroEncoders = new YoBoolean(name + "ZeroEncoders", registry);
      checkEncoderOffsets = new YoBoolean(name + "CheckEncoderOffsets", registry);
      checkEncoderOffsets.set(true);

      zeroEncoders.addListener(s ->
                               {
                                  if (zeroEncoders.getBooleanValue())
                                     beginZeroing();
                               });

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
               LogTools.warn(
                     "Attempted to set motor direction to " + attemptedValue + " which is not allowed. Motor direction can only be 1.0 or -1.0, resetting to "
                     + actualValue);
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

      statorTemp = new YoDouble("StatorTemp", registry);

      silTime = new YoDouble(prefix + "SILTime", registry);
      silDT = new YoDouble(prefix + "SILDT", registry);

      this.kt = new YoDouble(prefix + "kt", registry);
      this.kt.set(physicalParameters.getKt());

      this.maxAllowableStatorTemperature = new YoInteger(prefix + "maxAllowableStatorTemperature", registry);
      setMaxAllowableStatorTemperature(90);
      this.maxRecommendedStatorTemperature = new YoInteger(prefix + "maxRecommendedStatorTemperature", registry);
      setMaxRecommendedStatorTemperature(80);

      gearRatio = new YoDouble(prefix + "gearRatio", registry);
      gearRatio.set(physicalParameters.getGearRatio());

      inputCountsPerRevolution = physicalParameters.getCountsPerMotorRevolution();
      motorEncoderCountsToMotorRadians = (2.0 * Math.PI) / inputCountsPerRevolution;
      motorRadiansToMotorEncoderCounts = 1.0 / motorEncoderCountsToMotorRadians;
      motorEncoderCountsToOutputRadians = motorEncoderCountsToMotorRadians * physicalParameters.getGearRatio();

      outputCountsPerRevolution = physicalParameters.getCountsPerOutputRevolution();
      outputRadiansToMotorEncoderCounts = 1.0 / (motorEncoderCountsToOutputRadians);
      outputEncoderCountsToOutputRadians = (2.0 * Math.PI) / outputCountsPerRevolution;

      motorPositionOffset = new YoDouble(name + "MotorPositionOffset", registry);
      motorPositionOffset.set(motorOffset);
      outputPositionOffset = new YoDouble(name + "OutputPositionOffset", registry);
      outputPositionOffset.set(outputOffset);

      averageMotorPosition = new SimpleMovingAverageFilteredYoVariable(name + "AverageMotorPosition", 100, registry);
      averageOutputPosition = new SimpleMovingAverageFilteredYoVariable(name + "AverageOutputPosition", 100, registry);

      maxDriveCurrentMilliAmps = new YoLong(prefix + "MaxDriveCurrentMilliAmps", registry);

      //desireds
      desiredMotorPositionForImpedanceControl = new YoDouble(prefix + "desiredMotorPositionForImpedanceControl", registry);
      desiredMotorVelocityForImpedanceControl = new YoDouble(prefix + "desiredMotorVelocityForImpedanceControl", registry);
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

      enableCompensation = new YoBoolean(prefix + "enableCompensationCurrents", registry);
      dahlFrictionForce = new YoDouble(prefix + "dahlFrictionForce", registry); // Dahl Friction Force
      dahlSlope = new YoDouble(prefix + "dahlSlope", registry); // Dahl Slope (offset by + 1.0)                         
      linearDampingCompensationGain = new YoDouble(prefix + "linearDampingCompensationGain", registry); // Linear Damping Compensation
      // Do not allow compensation values below 0
      applyValueLimits(dahlFrictionForce, 0.0, Double.POSITIVE_INFINITY);
      applyValueLimits(dahlSlope, 0.0, Double.POSITIVE_INFINITY);
      applyValueLimits(linearDampingCompensationGain, 0.0, Double.POSITIVE_INFINITY);

      coggingOutputScalar = new YoDouble(prefix + "coggingOutputScalar", registry); // Cogging Output Scalar
      //Compensation Scalars should only be between 0 and 1
      applyValueLimits(coggingOutputScalar, 0.0, 1.0);

      if (enableCompensationAtStart)
      {
         enableCompensation.set(true);
         coggingOutputScalar.set(silParameters.getCoggingOutputScalar());
         dahlFrictionForce.set(silParameters.getDahlFrictionForceGain());
         dahlSlope.set(silParameters.getDahlFrictionSlope());
         linearDampingCompensationGain.set(silParameters.getLinearDampingCompensationGain());
      }

      enableCompensation.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            if (enableCompensation.getBooleanValue())
            {
               coggingOutputScalar.set(silParameters.getCoggingOutputScalar());
               dahlFrictionForce.set(silParameters.getDahlFrictionForceGain());
               dahlSlope.set(silParameters.getDahlFrictionSlope());
               linearDampingCompensationGain.set(silParameters.getLinearDampingCompensationGain());
            }
            else
            {
               coggingOutputScalar.set(0.0);
               dahlFrictionForce.set(0.0);
               dahlSlope.set(0.0);
               linearDampingCompensationGain.set(0.0);
            }
         }
      });

      // Tunable thresholds for under and over volt protection
      softwareBasedOverVoltThreshold = new YoDouble(prefix + "SoftwareBasedOverVoltThreshold", registry);
      softwareBasedOverVoltThreshold.set(DEFAULT_SOFTWARE_BASED_OVER_FAULT_THRESHOLD);
      softwareBasedUnderVoltThreshold = new YoDouble(prefix + "SoftwareBasedUnderVoltThreshold", registry);
      softwareBasedUnderVoltThreshold.set(DEFAULT_SOFTWARE_BASED_UNDER_FAULT_THRESHOLD);

      // SIL Acceleration Integration Variables
      impedanceControlStiffness = new YoDouble(prefix + "ImpedanceControl_Stiffness", registry);
      impedanceControlDamping = new YoDouble(prefix + "ImpedanceControl_Damping", registry);
      impedanceControlMaxPositionError = new YoDouble(prefix + "ImpedanceControl_MaxPositionError", registry);
      impedanceControlMaxVelocityError = new YoDouble(prefix + "ImpedanceControl_MaxVelocityError", registry);

      // SIL debugging variables
      sil_linearDampingCompensationCurrent = new YoDouble(prefix + "sil_linearDampingCompensationCurrent", registry);
      sil_coggingCompensationMotorCurrent = new YoDouble(prefix + "sil_coggingCompensationMotorCurrent", registry);
      sil_dahlFrictionCompensationCurrent = new YoDouble(prefix + "sil_dahlFrictionCompensationCurrent", registry);
      sil_impedanceControlMotorFeedbackCurrent = new YoDouble(prefix + "sil_impedanceControlMotorFeedbackCurrent", registry);
      sil_feedForwardCurrent = new YoDouble(prefix + "sil_feedForwardCurrent", registry);
      sil_totalDesiredCurrent = new YoDouble(prefix + "sil_totalDesiredCurrent", registry);

      // SIL Socket error and warning status signals
      inputEncoderWarningValue = new YoDouble(prefix + "sil_inputEncoderWarningValue", registry);
      inputEncoderErrorValue = new YoDouble(prefix + "sil_inputEncoderErrorValue", registry);
      inputEncoderState = new YoEnum<>(prefix + "InputEncoderState", registry, EncoderState.class);

      outputEncoderWarningValue = new YoDouble(prefix + "sil_outputEncoderWarningValue", registry);
      outputEncoderErrorValue = new YoDouble(prefix + "sil_outputEncoderErrorValue", registry);
      outputEncoderState = new YoEnum<>(prefix + "OutputEncoderState", registry, EncoderState.class);

      //actuals
      rawMotorPositionFromTwitters = new YoDouble(prefix + "rawMotorPositionFromTwitters", registry);
      rawMotorVelocityFromTwitters = new YoDouble(prefix + "rawMotorVelocityFromTwitters", registry);
      measuredMotorPosition = new YoDouble(prefix + "measuredMotorPosition", registry);
      measuredMotorVelocity = new YoDouble(prefix + "measuredMotorVelocity", registry);
      filteredMotorPosition = new YoDouble(prefix + "filteredMotorPosition", registry);
      filteredMotorVelocity = new YoDouble(prefix + "filteredMotorVelocity", registry);
      measuredMotorVelocityFD = new YoDouble(prefix + "measuredMotorVelocityFD", registry);
      useFDforMotorVelocity = new YoBoolean(prefix + "useFDforMotorVelocity", registry);
      useFDforMotorVelocity.set(false);

      measuredOutputPositionFromMotor = new YoDouble(prefix + "measuredOutputPositionFromMotor", registry);
      measuredOutputVelocityFromMotor = new YoDouble(prefix + "measuredOutputVelocityFromMotor", registry);
      filteredOutputPositionFromMotor = new YoDouble(prefix + "filteredOutputPositionFromMotor", registry);
      filteredOutputVelocityFromMotor = new YoDouble(prefix + "filteredOutputVelocityFromMotor", registry);

      measuredOutputPosition = new YoDouble(prefix + "measuredOutputPosition", registry);
      measuredOutputVelocity = new YoDouble(prefix + "measuredOutputVelocity", registry);
      filteredOutputPosition = new YoDouble(prefix + "filteredOutputPosition", registry);
      filteredOutputVelocity = new YoDouble(prefix + "filteredOutputVelocity", registry);
      measuredOutputVelocityFD = new YoDouble(prefix + "measuredOutputVelocityFD", registry);
      useFDforOutputVelocity = new YoBoolean(prefix + "useFDforOutputVelocity", registry);
      useFDforOutputVelocity.set(false);

      measuredMotorCurrent = new YoDouble(prefix + "measuredMotorCurrent", registry);
      estimatedMotorTorque = new YoDouble(prefix + "estimatedMotorTorque", registry);
      estimatedOutputTorque = new YoDouble(prefix + "estimatedOutputTorque", registry);

      measuredAnalogInput2InADCCounts = new YoDouble(prefix + "MeasuredAnalogInput2InADCCounts", registry);
      measuredAnalogInput1InADCCounts = new YoDouble(prefix + "MeasuredAnalogInput1InADCCounts", registry);
      measuredAnalogInput2InVolts = new YoDouble(prefix + "MeasuredAnalogInput2InVolts", registry);
      measuredAnalogInput1InVolts = new YoDouble(prefix + "MeasuredAnalogInput1InVolts", registry);

      useAnalog1ForStatorTemp = new YoBoolean(prefix + "useAnalog1ForStatorTemp", registry);
      useAnalog1ForStatorTemp.set(USE_ANALOG_1_FOR_STATOR_TEMP);

      measuredAnalogInput1InADCCounts.addListener(change -> measuredAnalogInput1InVolts.set(convertAnalogInput1FromCountsToVolts(measuredAnalogInput1InADCCounts.getDoubleValue())));
      measuredAnalogInput2InADCCounts.addListener(change -> measuredAnalogInput2InVolts.set(convertAnalogInput2FromCountsToVolts(measuredAnalogInput2InADCCounts.getDoubleValue())));

      //actuals in raw units
      rawMeasuredMotorCurrent = new YoInteger(prefix + "rawMeasuredMotorCurrent", registry);

      //settings and op stuff
      requestedModeOfOperation = new YoEnum<>(prefix + "requestedModeOfOperation", registry, ElmoModeOfOperation.class);
      previousModeOfOperation = new YoEnum<>(prefix + "previousModeOfOperation", registry, ElmoModeOfOperation.class);
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
      //      elmoErrorString = new YoEnum<>(prefix + "elmoErrorString", "", registry, true, ElmoErrorCodes.EC);
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

      etherCATState = new YoEnum<>(prefix + "_EC_State", registry, State.class);

      useOutputVelocityFromMotor = new YoBoolean(prefix + "UseOutputVelocityFromInput", registry);
      useOutputPositionFromMotor = new YoBoolean(prefix + "UseOutputPositionFromInput", registry);

      outputEncoderInverted = new YoBoolean(prefix + "outputEncoderIsInverted", registry);
      offsetFromZero = 0.0;

      DRIVE_FAULTED.addListener(source ->
                                {
                                   if (DRIVE_FAULTED.getBooleanValue())
                                      MOTOR_FAULT.set(true);
                                });

      etherCATState.addListener(source ->
                                {
                                   if (etherCATState.getEnumValue() == State.OFFLINE)
                                      LogTools.error(getName() + " just went OFFLINE");
                                });

      requestedModeOfOperation.set(ElmoModeOfOperation.CYCLIC_SYNCHRONOUS_TORQUE);

      driveTemperature = new YoDouble(prefix + "silTemp", registry);

      motorPositionBreakFrequency = new YoDouble(prefix + "motorPositionBreakFrequency", registry);
      outputPositionBreakFrequency = new YoDouble(prefix + "outputPositionBreakFrequency", registry);
      motorVelocityBreakFrequency = new YoDouble(prefix + "motorVelocityBreakFrequency", registry);
      outputVelocityBreakFrequency = new YoDouble(prefix + "outputVelocityBreakFrequency", registry);
      motorPositionBreakFrequency.set(DEFAULT_MOTOR_POSITION_BREAK_FREQUENCY);
      outputPositionBreakFrequency.set(DEFAULT_OUTPUT_POSITION_BREAK_FREQUENCY);
      motorVelocityBreakFrequency.set(DEFAULT_MOTOR_VELOCITY_BREAK_FREQUENCY);
      outputVelocityBreakFrequency.set(DEFAULT_OUTPUT_VELOCITY_BREAK_FREQUENCY);
      platinumTwitter.setMotorPositionBreakFrequency(DEFAULT_MOTOR_POSITION_BREAK_FREQUENCY);
      platinumTwitter.setOutputPositionBreakFrequency(DEFAULT_OUTPUT_POSITION_BREAK_FREQUENCY);
      platinumTwitter.setMotorVelocityBreakFrequency(DEFAULT_MOTOR_VELOCITY_BREAK_FREQUENCY);
      platinumTwitter.setOutputVelocityBreakFrequency(DEFAULT_OUTPUT_VELOCITY_BREAK_FREQUENCY);

      parentRegistry.addChild(registry);
   }

   @Override
   public void read()
   {
      // estimate the update dt. We're finite differencing the velocity, so making sure this estimate is accurate is really important. If we don't have a
      // previous update time, then we can fall back to the provided dt.
      etherCATState.set(platinumTwitter.getState());
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
      double prevSILTime = silTime.getDoubleValue();
//      silTime.set(platinumTwitter.getSILControlTime());
      silDT.set(silTime.getDoubleValue() - prevSILTime);

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
      previousModeOfOperation.set(currentModeOfOperation.getEnumValue());
      int mode = platinumTwitter.getModeOfOperation();
      if (mode < -3 || mode > 11 || mode == -1 || mode == -2)
      {
         LogTools.warn("Mode of operation from " + getName() + " is a reserved mode, can't use " + mode);
         mode = 0;
      }
      currentModeOfOperation.set(mode);

      statorTemp.set(getStatorTemperature());

      // Update SIL readable variables
      sil_dahlFrictionCompensationCurrent.set(platinumTwitter.getSILDahlFrictionCompensationCurrent());
      sil_linearDampingCompensationCurrent.set(platinumTwitter.getSILLinearDampingCompensationCurrent());
      sil_coggingCompensationMotorCurrent.set(platinumTwitter.getSILDesiredCoggingCompensationCurrent());
      sil_impedanceControlMotorFeedbackCurrent.set(platinumTwitter.getSILDesiredPDControlFeedbackCurrent());

      sil_feedForwardCurrent.set(platinumTwitter.getSILDesiredFeedForwardCurrent());
      sil_totalDesiredCurrent.set(platinumTwitter.getSILDesiredTotalCurrent());

//      driveTemperature.set(platinumTwitter.getSILTemperature());

      inputEncoderWarningValue.set(platinumTwitter.getSocket1Warning());
      inputEncoderErrorValue.set(platinumTwitter.getSocket1Error());
      outputEncoderWarningValue.set(platinumTwitter.getSocket2Warning());
      outputEncoderErrorValue.set(platinumTwitter.getSocket2Error());

      updateEncoderStates();

      measuredBusVoltage.set(platinumTwitter.getDCLinkVoltageMilliVolts() / 1000.0);

      // Update fault info
      UNDER_VOLTAGE.set(platinumTwitter.isUnderVoltage() || measuredBusVoltage.getDoubleValue() < softwareBasedUnderVoltThreshold.getDoubleValue());
      OVER_VOLTAGE.set(platinumTwitter.isOverVoltage() || measuredBusVoltage.getDoubleValue() > softwareBasedOverVoltThreshold.getDoubleValue());
      STO_DISABLED.set(platinumTwitter.isSTODisabled());
      CURRENT_SHORT.set(platinumTwitter.isCurrentShorted());
      OVER_TEMPERATURE.set(platinumTwitter.isOverTemperature());
      boolean isFaulted = UNDER_VOLTAGE.getBooleanValue()
                          || OVER_VOLTAGE.getBooleanValue()
                          || STO_DISABLED.getBooleanValue()
                          || CURRENT_SHORT.getBooleanValue()
                          || OVER_TEMPERATURE.getBooleanValue();
      DRIVE_FAULTED.set(isFaulted || platinumTwitter.isFaulted() || !platinumTwitter.isOperational());

      /** Motor Space Encoders **/
      // get the motor position on the previous tick. This is used to finite difference the motor position to get the motor velocity.
      double previousMeasuredMotorPosition = measuredMotorPosition.getDoubleValue();

      rawMotorPositionFromTwitters.set(platinumTwitter.getMeasuredMotorPosition());
      rawMotorVelocityFromTwitters.set(platinumTwitter.getMeasuredMotorVelocity());
      measuredMotorPosition.set(motorDirection.getDoubleValue() * (rawMotorPositionFromTwitters.getDoubleValue() - motorPositionOffset.getValue()));
      filteredMotorPosition.set(motorDirection.getDoubleValue() * (platinumTwitter.getFilteredMotorPosition() - motorPositionOffset.getValue()));
      measuredMotorVelocity.set(motorDirection.getDoubleValue() * rawMotorVelocityFromTwitters.getDoubleValue());
      filteredMotorVelocity.set(motorDirection.getDoubleValue() * platinumTwitter.getFilteredMotorVelocity());

      //project motor measureds in output space
      measuredOutputPositionFromMotor.set(measuredMotorPosition.getDoubleValue() / gearRatio.getDoubleValue());
      measuredOutputVelocityFromMotor.set(measuredMotorVelocity.getValue() / gearRatio.getDoubleValue());
      filteredOutputPositionFromMotor.set(filteredMotorPosition.getDoubleValue() / gearRatio.getDoubleValue());
      filteredOutputVelocityFromMotor.set(filteredMotorVelocity.getDoubleValue() / gearRatio.getDoubleValue());

      /** Output Space Encoders **/
      double previousMeasuredOutputPosition = measuredOutputPosition.getDoubleValue();

      double outputDirection = outputEncoderInverted.getBooleanValue() ? -motorDirection.getDoubleValue() : motorDirection.getDoubleValue();

      measuredOutputPosition.set(outputDirection * (platinumTwitter.getMeasuredOutputPosition() - outputPositionOffset.getValue()));
      filteredOutputPosition.set(outputDirection * (platinumTwitter.getFilteredOutputPosition() - outputPositionOffset.getValue()));
      measuredOutputVelocity.set(outputDirection * platinumTwitter.getMeasuredOutputVelocity());
      filteredOutputVelocity.set(outputDirection * platinumTwitter.getFilteredOutputVelocity());

      // finite difference the measured velocity, looking at the previous encoder measurement.
      measuredOutputVelocityFD.set((measuredOutputPosition.getDoubleValue() - previousMeasuredOutputPosition) / estimatedDt.getDoubleValue());

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
      measuredAnalogInput2InADCCounts.set(platinumTwitter.getMeasuredAnalogInput2());
      measuredAnalogInput1InADCCounts.set(platinumTwitter.getMeasuredAnalogInput1());

      encoderDifferenceAtOutput.set(measuredOutputPositionFromMotor.getDoubleValue() - measuredOutputPosition.getDoubleValue());
      if(zeroEncoders.getBooleanValue())
      {
         if(averageMotorPosition.getHasBufferWindowFilled())
         {
            motorPositionOffset.set(averageMotorPosition.getDoubleValue() - motorDirection.getDoubleValue() * offsetFromZero * gearRatio.getDoubleValue());
            outputPositionOffset.set(averageOutputPosition.getDoubleValue() - outputDirection * offsetFromZero);
            offsetFromZero = 0.0;
            zeroEncoders.set(false, false);
         }
         else
         {
            averageMotorPosition.update(platinumTwitter.getMeasuredMotorPosition());
            averageOutputPosition.update(platinumTwitter.getMeasuredOutputPosition());
         }
      }
      else if (!isDriveEnabled() && checkEncoderOffsets.getBooleanValue())
         checkAndUpdateEncoderOffsets(); // While the motor is disabled, check if the encoder isn't correct

   }

   /**
    * Update the current encoder state based on the current error value from the twitter
    */
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
      rawDesiredMotorPosition.set((int) (motorDirection.getDoubleValue() * ((desiredMotorPosition.getDoubleValue() + motorPositionOffset.getDoubleValue())
                                                                            * motorRadiansToMotorEncoderCounts)));

      // Set the desired motor velocity in encoder counts per second. Flip the sign if the directionality is reversed so that it matches the motor axis.
      rawDesiredMotorVelocity.set((int) (motorDirection.getDoubleValue() * (desiredMotorVelocity.getDoubleValue() * motorRadiansToMotorEncoderCounts)));

      // Compute the total desired current for the drive. This is the summation of the feedforward motor current, the desired motor current, which is the main
      // setpoint of this drive and comes from the desired motor torque, and the velocity feedforward current. This is likely the same as the desired
      // feedforward motor current.
      desiredFeedForwardMotorCurrent.set(0.0); //TODO Figure out how to do this properly
      //      desiredMotorTorque.set(desiredOutputTorque.getDoubleValue() / gearRatio.getDoubleValue());
      desiredOutputTorque.set(desiredMotorTorque.getDoubleValue() * gearRatio.getDoubleValue());
      desiredMotorCurrent.set(desiredMotorTorque.getDoubleValue() / kt.getDoubleValue());
      desiredMotorCurrentWithFF.set(desiredMotorCurrent.getDoubleValue() + desiredFeedForwardMotorCurrent.getDoubleValue());

      // Compute the desired motor current to an effort percentage, -1000 to 1000. This is a percentage of the maximum effort available from the drive. Flip the
      // sign of the current if the directionality is reversed so that it matches the motor axis.
      double desiredEffortPercentage =
            motorDirection.getDoubleValue() * EuclidCoreTools.clamp(desiredMotorCurrentWithFF.getDoubleValue() / platinumTwitter.getMaxDriveCurrentAmps(), 1.0);
      rawDesiredMotorEffortPercentage.set((int) (CURRENT_SIGNAL_RANGE * desiredEffortPercentage));

      // Set the actual objectives for the drive. This includes the desired motor encoder counts, the desired motor encoder counts per second, and the
      // desired percentage of max effort, -1000 to 1000.
//      platinumTwitter.setRawTargetPosition(rawDesiredMotorPosition.getIntegerValue());
//      platinumTwitter.setRawTargetVelocity(rawDesiredMotorVelocity.getIntegerValue());
      platinumTwitter.setPercentageMaxEffort(rawDesiredMotorEffortPercentage.getIntegerValue());

      // Set the desired SIL controller parameters to the amplifier.
      platinumTwitter.setDahlFrictionForce(dahlFrictionForce.getDoubleValue());
      platinumTwitter.setDahlSlope(dahlSlope.getDoubleValue());
      platinumTwitter.setLinearDampingCompensation(linearDampingCompensationGain.getDoubleValue());

      platinumTwitter.setCoggingOutputScalar(coggingOutputScalar.getDoubleValue());

      // Set acceleration Integration Parameters
      platinumTwitter.setMotorStiffnessForImpedanceControl(impedanceControlStiffness.getDoubleValue());
      platinumTwitter.setMotorDampingForImpedanceControl(impedanceControlDamping.getDoubleValue());

      desiredMotorPositionForImpedanceControl.set(motorDirection.getDoubleValue() * desiredMotorPosition.getDoubleValue() + motorPositionOffset.getDoubleValue());
      desiredMotorVelocityForImpedanceControl.set(motorDirection.getDoubleValue() * desiredMotorVelocity.getDoubleValue());
      platinumTwitter.setDesiredMotorPositionForImpedanceControl(desiredMotorPositionForImpedanceControl.getDoubleValue());
      platinumTwitter.setDesiredMotorVelocityForImpedanceControl(desiredMotorVelocityForImpedanceControl.getDoubleValue());

      platinumTwitter.setMaxMotorPositionError(impedanceControlMaxPositionError.getDoubleValue());
      platinumTwitter.setMaxMotorVelocityError(impedanceControlMaxVelocityError.getDoubleValue());

      platinumTwitter.setMotorPositionBreakFrequency(motorPositionBreakFrequency.getDoubleValue());
      platinumTwitter.setOutputPositionBreakFrequency(outputPositionBreakFrequency.getDoubleValue());
      platinumTwitter.setMotorVelocityBreakFrequency(motorVelocityBreakFrequency.getDoubleValue());
      platinumTwitter.setOutputVelocityBreakFrequency(outputVelocityBreakFrequency.getDoubleValue());

      platinumTwitter.doStateControl();
   }

   /**
    * Apply limits to a {@code YoDouble} to bound possible values to [lowerLimit, upperLimit]
    *
    * @param variableToLimit variable to be limited
    * @param lowerLimit      Lower value limit, inclusive
    * @param upperLimit      Upper value limit, inclusiv
    */
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

   /**
    * Clear all possible faults on the twitter by setting them to false
    */
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
   }

   /**
    * Check to see if the output encoder offset is off by less than pi, since anything more than pi means the encoder signal
    * shifted by 2*pi. If off by more, update the offset until it is less than pi
    */
   private void checkAndUpdateOutputOffset()
   {
      double fullRotation = 2 * Math.PI;
      double difference = measuredOutputPosition.getDoubleValue();
      int rotationInterval = (int) Math.floor(Math.abs(difference) / fullRotation);
      double outputDirection = getOutputDirection();
      if (difference >= (fullRotation / 2.0))
      {
         if (rotationInterval == 0)
            outputPositionOffset.add(outputDirection * fullRotation);
         else
            outputPositionOffset.add(outputDirection * rotationInterval * fullRotation);
      }
      if (difference <= -(fullRotation / 2.0))
      {
         if (rotationInterval == 0)
            outputPositionOffset.sub(outputDirection * fullRotation);
         else
            outputPositionOffset.sub(outputDirection * rotationInterval * fullRotation);
      }
      measuredOutputPosition.set(outputDirection * (platinumTwitter.getMeasuredOutputPosition() - outputPositionOffset.getDoubleValue()));
   }

   /**
    * Check if the output position estimated from the input encoder is within pi/gearRatio of the output position.
    * If not, updates the input encoder offset to be within range.
    */
   private void checkAndUpdateMotorOffset()
   {
      double fullRotation = 2 * Math.PI;
      double maxDifference = Math.PI / gearRatio.getDoubleValue();
      int rotationInterval = (int) Math.abs(encoderDifferenceAtOutput.getDoubleValue() / (maxDifference * 2));
      if (encoderDifferenceAtOutput.getDoubleValue() >= maxDifference)
      {
         if (rotationInterval == 0)
            motorPositionOffset.add(motorDirection.getDoubleValue() * fullRotation);
         else
            motorPositionOffset.add(motorDirection.getDoubleValue() * rotationInterval * fullRotation);
      }
      if (encoderDifferenceAtOutput.getDoubleValue() <= -maxDifference)
      {
         if (rotationInterval == 0)
            motorPositionOffset.sub(motorDirection.getDoubleValue() * fullRotation);
         else
            motorPositionOffset.sub(motorDirection.getDoubleValue() * rotationInterval * fullRotation);
      }
   }

   /**
    * Check the encoder offsets and update them if necessary
    */
   public void checkAndUpdateEncoderOffsets()
   {
      checkAndUpdateOutputOffset();
      checkAndUpdateMotorOffset();
   }

   /**
    * @param voltage Voltage from the temperature sensor
    * @return The temperature of the cycloid, converted from voltage to degrees Celsius
    */
   public double convertAnalogInputInVoltsToTemperatureInDegreeCelsius(double voltage)
   {
      return (TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[0] * voltage * voltage + TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[1] * voltage
              + TEMPERATURE_VOLTAGE_FUNCTION_COEFFECIENTS[2]);
   }

   public double convertAnalogInput1FromCountsToVolts(double valueInCounts)
   {
      return valueInCounts * ANALOG_INPUT_1_CONVERSION_CONSTANT_FROM_NADIA;
   }

   public double convertAnalogInput2FromCountsToVolts(double valueInCounts)
   {
      return valueInCounts * ANALOG_INPUT_2_CONVERSION_CONSTANT_FROM_NADIA;
   }

   public void zeroEncodersWithOffset(double offsetFromZero)
   {
      this.offsetFromZero = offsetFromZero;
      zeroEncoders.set(true);
   }

   private void beginZeroing()
   {
      averageMotorPosition.reset();
      averageOutputPosition.reset();
   }

   public void setMotorDirection(boolean isMotorDirectionReversed)
   {
      if (isMotorDirectionReversed)
         motorDirection.set(-1.0);
      else
         motorDirection.set(1.0);
   }

   public void setEnableCompensationCurrents(boolean enableCompensationCurrents)
   {
      this.enableCompensation.set(enableCompensationCurrents);
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
      desiredMotorPosition.set(motorPosition);
   }

   @Override
   public void setDesiredMotorVelocity(double motorVelocity)
   {
      desiredMotorVelocity.set(motorVelocity);
   }

   public void setKt(double kt)
   {
      this.kt.set(kt);
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

   @Override
   public void setDesiredMotorStiffness(double desiredMotorStiffness)
   {
      if (Double.isFinite(desiredMotorStiffness) && desiredMotorStiffness >= 0)
         impedanceControlStiffness.set(desiredMotorStiffness);
      else if (desiredMotorStiffness < 0)
      {
         LogTools.warn("Tried to set negative stiffness to " + getName() + ", setting stiffness to 0");
         impedanceControlStiffness.set(0.0);
      }
      else
         LogTools.warn("Tried to set stiffness at " + getName() + " to a non-finite value of " + desiredMotorStiffness);
   }

   @Override
   public void setDesiredMotorDamping(double desiredMotorDamping)
   {
      if (Double.isFinite(desiredMotorDamping) && desiredMotorDamping >= 0)
         impedanceControlDamping.set(desiredMotorDamping);
      else if (desiredMotorDamping < 0)
      {
         LogTools.error("Tried to set negative damping to " + getName() + ", setting damping to 0");
         impedanceControlDamping.set(0.0);
      }
   }

   @Override
   public void setMaxPositionFeedbackError(double maxPositionFeedbackError)
   {
      if (Double.isFinite(maxPositionFeedbackError))
         impedanceControlMaxPositionError.set(Math.abs(maxPositionFeedbackError));
      else
         LogTools.warn("Tried to set max position feedback error to " + maxPositionFeedbackError + ", which is not a valid input");
   }

   @Override
   public void setMaxVelocityFeedbackError(double maxVelocityFeedbackError)
   {
      if (Double.isFinite(maxVelocityFeedbackError))
         impedanceControlMaxVelocityError.set(Math.abs(maxVelocityFeedbackError));
      else
         LogTools.warn("Tried to set max velocity feedback error to " + maxVelocityFeedbackError + ", which is not a valid input");
   }

   public void setUseOutputPositionFromMotor(boolean useOutputPositionFromMotor)
   {
      this.useOutputPositionFromMotor.set(useOutputPositionFromMotor);
   }

   public void setUseOutputVelocityFromMotor(boolean useOutputVelocityFromMotor)
   {
      this.useOutputVelocityFromMotor.set(useOutputVelocityFromMotor);
   }

   public void setMaxAllowableStatorTemperature(int maxAllowableStatorTemperature)
   {
      if (maxAllowableStatorTemperature > 0)
         this.maxAllowableStatorTemperature.set(maxAllowableStatorTemperature);
      else
         LogTools.warn("Tried to set max allowable stator temperature to " + maxAllowableStatorTemperature + ", which is not a valid input");
   }

   public void setMaxRecommendedStatorTemperature(int maxRecommendedStatorTemperature)
   {
      if (maxRecommendedStatorTemperature > 0)
         this.maxRecommendedStatorTemperature.set(maxRecommendedStatorTemperature);
      else
         LogTools.warn("Tried to set max recommended stator temperature to " + maxAllowableStatorTemperature + ", which is not a valid input");
   }

   @Override
   public double getEncoderDifferenceAtOutput()
   {
      return useOutputPositionFromMotor.getBooleanValue() ? 0.0 : encoderDifferenceAtOutput.getDoubleValue();
   }

   @Override
   public boolean isMotorFaulted()
   {
      return MOTOR_FAULT.getBooleanValue();
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

   public double getMaxActuatorTorque()
   {
      return kt.getDoubleValue() * gearRatio.getDoubleValue() * platinumTwitter.getMaxDriveCurrentAmps();
   }

   private double getOutputDirection()
   {
      return outputEncoderInverted.getBooleanValue() ? -motorDirection.getDoubleValue() : motorDirection.getDoubleValue();
   }

   @Override
   public double getMeasuredMotorPosition()
   {
      return measuredMotorPosition.getDoubleValue();
   }

   @Override
   public double getFilteredMotorPosition()
   {
      return  filteredMotorPosition.getDoubleValue();
   }

   @Override
   public double getMeasuredOutputPosition()
   {
      return useOutputPositionFromMotor.getBooleanValue() ? measuredOutputPositionFromMotor.getDoubleValue() : measuredOutputPosition.getDoubleValue();
   }

   @Override
   public double getFilteredOutputPosition()
   {
      return useOutputPositionFromMotor.getBooleanValue() ? filteredOutputPositionFromMotor.getDoubleValue() : filteredOutputPosition.getDoubleValue();
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
   public double getMeasuredOutputVelocity()
   {
      return useOutputVelocityFromMotor.getBooleanValue() ? measuredOutputVelocityFromMotor.getDoubleValue() : measuredOutputVelocity.getValue();
   }

   @Override
   public double getFilteredOutputVelocity()
   {
      return useOutputVelocityFromMotor.getBooleanValue() ? filteredOutputVelocityFromMotor.getDoubleValue() : filteredOutputVelocity.getDoubleValue();
   }

   public double getMeasuredMotorCurrent()
   {
      return measuredMotorCurrent.getDoubleValue();
   }

   @Override
   public State getEtherCATState()
   {
      return etherCATState.getEnumValue();
   }

   public CycloidPhysicalParameters getPhysicalParameters()
   {
      return physicalParameters;
   }

   public double getStatorTemperature()
   {
      if (useAnalog1ForStatorTemp.getBooleanValue())
         return convertAnalogInputInVoltsToTemperatureInDegreeCelsius(measuredAnalogInput1InVolts.getValue());
      else
         return convertAnalogInputInVoltsToTemperatureInDegreeCelsius(measuredAnalogInput2InVolts.getValue());
   }

   public int getMaxAllowableStatorTemperature()
   {
      return this.maxAllowableStatorTemperature.getValue();
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

   public boolean isDriveEnabled()
   {
      return enableDrive.getBooleanValue();
   }

   public boolean isDynamicBrakingEnabled()
   {
      return dynamicBrakingEnabled.getBooleanValue();
   }

   public String getActuatorPackage()
   {
      return actuatorPackage;
   }

   public CycloidPlatinumTwitter getPlatinumTwitter()
   {
      return platinumTwitter;
   }

   public void setMotorPositionBreakFrequency(double breakFrequency)
   {
      motorPositionBreakFrequency.set(breakFrequency);
   }

   public void setOutputPositionBreakFrequency(double breakFrequency)
   {
      outputPositionBreakFrequency.set(breakFrequency);
   }

   public void setMotorVelocityBreakFrequency(double breakFrequency)
   {
      motorVelocityBreakFrequency.set(breakFrequency);
   }

   public void setOutputVelocityBreakFrequency(double breakFrequency)
   {
      outputVelocityBreakFrequency.set(breakFrequency);
   }

   public void setOutputEncoderInverted(boolean outputEncoderInverted)
   {
      this.outputEncoderInverted.set(outputEncoderInverted);
   }
}