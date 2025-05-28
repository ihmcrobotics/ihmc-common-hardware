package us.ihmc.devices.cycloids;

import us.ihmc.devices.etherCATDevices.elmo.ElmoTwitterStatusRegisterProcessor;
import us.ihmc.etherCAT.slaves.DSP402Slave;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;

public class LowLevelCycloidManager
{
   private final CycloidPlatinumTwitter twitter;
   
   private double motorEncoderCountsToMotorRadians;
   private double motorEncoderCountsToOutputRadians;

   private double outputEncoderCountsToOutputRadians;
   private double outputRadiansToMotorEncoderCounts;

   protected double desiredMotorPosition;
   protected double desiredMotorVelocity;
   protected double desiredFeedForwardMotorCurrent;
   protected double desiredMotorCurrent;
   protected double desiredMotorCurrentWithFF;
   protected double desiredMotorTorque;
   protected double desiredOutputTorque;

   protected int rawDesiredMotorPosition;
   protected int rawDesiredMotorVelocity;
   protected int rawDesiredMotorEffortPercentage;

   protected boolean[] digitalOutputs = new boolean[6];
   protected ElmoModeOfOperation requestedModeOfOperation;
   protected ElmoModeOfOperation currentModeOfOperation;

   // measured variables
   private double measuredMotorPosition;
   private double measuredMotorVelocity;
   private double measuredMotorVelocityFD;
   private boolean useFDforMotorVelocity;

   private double measuredMotorPositionInOutput;
   private double measuredMotorVelocityInOutput;

   private double measuredOutputPosition;
   private double measuredOutputVelocity;
   private double measuredOutputVelocityFD;
   private boolean useFDforOutputVelocity;

   private long maxDriveCurrentMilliAmps;
   private double measuredMotorCurrent;
   private double estimatedMotorTorque;
   private double estimatedOutputTorque;

   private int rawMeasuredMotorPosition;
   private double rawMeasuredMotorVelocity;
   private double rawMeasuredOuputPosition;
   private double rawMeasuredOutputVelocity;
   private int rawMeasuredMotorCurrent;

   private DSP402Slave.StatusWord statusWord;
   private long elmoStatusRegister;
   private int errorCode;
   private double measuredBusVoltage;
   private double measuredAnalogInput2;

   // error variables
   private ElmoTwitterStatusRegisterProcessor statusRegisterProcessor;
   private boolean DRIVE_FAULTED;
   private boolean UNDER_VOLTAGE;
   private boolean OVER_VOLTAGE;
   private boolean STO_DISABLED;
   private boolean CURRENT_SHORT;
   private boolean OVER_TEMPERATURE;
   private boolean MOTOR_ENABLED;
   private boolean MOTOR_FAULT;
   private boolean CURRENT_LIMITED;

   // SIL Tunable Variables
   private boolean saveR2ToNVM; //Save R2 to NVM, this can only be done once per power cycle

   private double dahlFrictionForce; // Dahl Friction Force
   private double dahlSlope; // Dahl Slope (offset by + 1.0)
   private double linearDampingCompensation; // Linear Damping Compensation
   private double dahlOutputScalar; // Dahl Output Scalar
   private double linearDampingOutputScalar; // Linear Damping Output Scalar
   private double coggingOutputScalar; // Cogging Output Scalar

   // SIL Acceleration Integration Variables
   private double accelerationIntegrationDesiredMotorPosition;
   private double accelerationIntegrationDesiredMotorVelocity;
   private double accelerationIntegrationStiffness;
   private double accelerationIntegrationDamping;
   private double accelerationIntegrationScalar;
   private double accelerationIntegrationMaxPositionError;
   private double accelerationIntegrationMaxVelocityError;

   // Control variables
   private boolean enableDrive;
   private boolean enableCompensation;
   private boolean clearFaults;
   private boolean stayDisabled;
   private DSP402Slave.ControlWord controlWord;

   private double kt;
   private int maxAllowableStatorTemperature;
   private int maxRecommendedStatorTemperature;
   private double motorDirection;
   private double gearRatio;

   private final CycloidActuatorParameters actuatorParameters;
   private final SILParameters silParameters;

   // SIL Debuggging variables
   private double sil_linearDampingCompensationCurrent;
   private double sil_coggingCompensationMotorCurrent;
   private double sil_dahlFrictionCompensationCurrent;
   private double sil_acceleratiojnIntegrationMotorFeedbackCurrent;
   private double sil_accelerationIntegrationMeasuredMotorPosition;
   private double sil_accelerationIntegrationMeasuredMotorVelocity;

   // SIL Socket warning and error status variables
   private double inputEncoderWarningValue;
   private double inputEncoderErrorValue;
   private double outputEncoderWarningValue;
   private double outputEncoderErrorValue;

   private enum EncoderState
   {
      NORMAL, WARNING, ERROR
   }

   private EncoderState inputEncoderState;
   private EncoderState outputEncoderState;

   private final CycloidActuatorPackage actuatorPackage;

   private boolean reverseMotorDirection;
   //   private double motorDirection = 1.0;
   private double zeroPositionOffset;

   public LowLevelCycloidManager(CycloidPlatinumTwitter twitter, CycloidActuatorPackage actuatorPackage)
   {
      this.twitter = twitter;
      this.actuatorPackage = actuatorPackage;

      this.actuatorParameters = CycloidActuatorParameters.createCycloidParameters(actuatorPackage);
      this.silParameters = SILParameters.createParameters(actuatorPackage);
   }
}
