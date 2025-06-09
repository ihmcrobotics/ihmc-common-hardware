package us.ihmc.devices.cycloids;

import us.ihmc.devices.etherCATDevices.elmo.PlatinumTwitter;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.hardwareStatusUI.controllerSide.EtherCATDeviceStatusProvider;
import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;

public class CycloidPlatinumTwitter extends PlatinumTwitter implements EtherCATDeviceStatusProvider
{
   private static final int MAX_CURRENT_ADDRESS = 0x6075;
   private static final int MAX_TORQUE_ADDRESS = 0x6076;

   private final RPDO_1600 rpdo_1600 = new RPDO_1600();
   private final RPDO_1601 rpdo_1601 = new RPDO_1601();
   private final RPDO_1602 rpdo_1602 = new RPDO_1602();
   private final TPDO_1a00 tpdo_1a00 = new TPDO_1a00();
   private final TPDO_1a01 tpdo_1a01 = new TPDO_1a01();
   private final TPDO_1a02 tpdo_1a02 = new TPDO_1a02();

   private long maxDriveCurrentMilliAmps;
   private double maxDriveCurrentAmps;
   private double ampsPerCurrentCount;
   private double currentCountPerAmp;

   static final double AMPS_PER_MILLIAMPS = 1.0e-3;

   public class RPDO_1600 extends RxPDO
   {
      public RPDO_1600()
      {
         super(0x1600);
      }

      Unsigned16 controlWord = new Unsigned16(); //CW[1] (0x6040) Control Word
      Unsigned8 modeOfOperation = new Unsigned8(); //(0x6060) Mode of Operation
      Signed32 targetPosition = new Signed32(); //PA[1] (0x607A) Target position (Absolute)
      Signed32 targetVelocity = new Signed32(); //JV (0x60FF) Target Velocity
      Signed32 positionOffset = new Signed32(); //(0x60B0) Position Offset
      Signed16 targetTorquePercentage = new Signed16(); //(0x6071) Target torque -1000 (-100%) to 1000 (100%)
      Signed32 velocityOffset = new Signed32(); //(0x60B1) Velocity Offset
   }

   private void configurePDO1600()
   {
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1600, 1, computePdoMapValue(0x6040, 0, 16)), "failed to write to 0x1600 - 0x6040"); // control word
      verifyWorkingCounter(writeSDO(0x1600, 2, computePdoMapValue(0x6060, 0, 8)), "failed to write to 0x1600 - 0x6060"); // mode of operation
      verifyWorkingCounter(writeSDO(0x1600, 3, computePdoMapValue(0x607A, 0, 32)), "failed to write to 0x1600 - 0x607A"); // target position
      verifyWorkingCounter(writeSDO(0x1600, 4, computePdoMapValue(0x60FF, 0, 32)), "failed to write to 0x1600 - 0x60FF"); // target velocity
      verifyWorkingCounter(writeSDO(0x1600, 5, computePdoMapValue(0x60B0, 0, 32)), "failed to write to 0x1600 - 0x60B0"); // position offset
      verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(0x6071, 0, 16)), "failed to write to 0x1600 - 0x6071"); // torque demand
      verifyWorkingCounter(writeSDO(0x1600, 7, computePdoMapValue(0x60B1, 0, 32)), "failed to write to 0x1600 - 0x60B1"); // velocity offset

      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 7), "failed to write to 0x1600 - 0x7"); // num elements in 0x1600 (max 8)
   }

   public class RPDO_1601 extends RxPDO
   {
      public RPDO_1601()
      {
         super(0x1601);
      }

      Signed32 saveR2ToNVM = new Signed32(); //Save R2 to NVM R1[1] (0x22F3 1)

      Float64 dahlFrictionForce = new Float64(); // Dahl Friction Force R2[1] (0x22F4 1)
      Float64 dahlSlope = new Float64(); // Dahl Slope (offset by + 1.0) R2[2] (0x22F4 2)
      Float64 linearDampingCompensation = new Float64(); // Linear Damping Compensation R2[3] (0x22F4 3)
      Float64 dahlOutputScalar = new Float64(); // Dahl Output Scalar R2[10] (0x22F4 10)
      Float64 linearDampingOutputScalar = new Float64(); // Linear Damping Output Scalar R2[11] (0x22F4 11)
      Float64 coggingCompensationScalar = new Float64(); // Cogging Compensation Output Scalar R2[12] (0x22F4 12)
   }

   private void configurePDO1601()
   {
      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 0), "failed to write to 0x1601 - 0x0"); // disable 0x1601 while we write

      verifyWorkingCounter(writeSDO(0x1601, 1, computePdoMapValue(0x22F3, 2, 32)), "failed to write to 0x1601 - 0x22F3 1"); // R1[1]
      verifyWorkingCounter(writeSDO(0x1601, 2, computePdoMapValue(0x22F4, 1, 64)), "failed to write to 0x1601 - 0x22F4 1"); //  R2[1]
      verifyWorkingCounter(writeSDO(0x1601, 3, computePdoMapValue(0x22F4, 2, 64)), "failed to write to 0x1601 - 0x22F4 2"); // R2[2]
      verifyWorkingCounter(writeSDO(0x1601, 4, computePdoMapValue(0x22F4, 3, 64)), "failed to write to 0x1601 - 0x22F4 3"); // R2[3]
      verifyWorkingCounter(writeSDO(0x1601, 5, computePdoMapValue(0x22F4, 10, 64)), "failed to write to 0x1601 - 0x22F4 10"); // R2[10]
      verifyWorkingCounter(writeSDO(0x1601, 6, computePdoMapValue(0x22F4, 11, 64)), "failed to write to 0x1601 - 0x22F4 11"); // R2[11]
      verifyWorkingCounter(writeSDO(0x1601, 7, computePdoMapValue(0x22F4, 12, 64)), "failed to write to 0x1601 - 0x22F4 12"); // R2[12]

      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 7), "failed to write to 0x1601 - 0x8"); // num elements in 0x1601 (max 8)
   }

   public class RPDO_1602 extends RxPDO
   {
      public RPDO_1602()
      {
         super(0x1602);
      }

      Float64 desiredMotorPositionInRadians = new Float64(); // Controller set desired Motor position R2[51] (0x22F4 51)
      Float64 desiredMotorVelocityInRadians = new Float64(); // Controller set desired Motor velocity R2[52] (0x22F4 52)
      Float64 motorStiffness = new Float64(); // Stiffness (Kp) for position-based torque control R2[55] (0x22F4 55)
      Float64 motorDamping = new Float64(); // Damping (Kd) for velocity-based torque control R2[56] (0x22F4 56)
      Float64 maxMotorFeedbackPositionError = new Float64(); // R2[53] (0x22F4 53)
      Float64 maxMotorFeedbackVelocityError = new Float64(); // R2[54] (0x22F4 54)
      Float64 accelerationIntegrationScalar = new Float64(); // Position and velocity FeedBack Scalar R2[13] (0x22F4 13)
   }

   private void configurePDO1602()
   {
      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 0), "failed to write to 0x1602 - 0x0"); // disable 0x1601 while we write

      verifyWorkingCounter(writeSDO(0x1602, 1, computePdoMapValue(0x22F4, 51, 64)), "failed to write to 0x1602 - 0x22F4 51"); // R2[51]
      verifyWorkingCounter(writeSDO(0x1602, 2, computePdoMapValue(0x22F4, 52, 64)), "failed to write to 0x1602 - 0x22F4 52"); // R2[52]
      verifyWorkingCounter(writeSDO(0x1602, 3, computePdoMapValue(0x22F4, 55, 64)), "failed to write to 0x1602 - 0x22F4 55"); // R2[55]
      verifyWorkingCounter(writeSDO(0x1602, 4, computePdoMapValue(0x22F4, 56, 64)), "failed to write to 0x1602 - 0x22F4 56"); // R2[56]
      verifyWorkingCounter(writeSDO(0x1602, 5, computePdoMapValue(0x22F4, 53, 64)), "failed to write to 0x1602 - 0x22F4 53"); // R2[53]
      verifyWorkingCounter(writeSDO(0x1602, 6, computePdoMapValue(0x22F4, 54, 64)), "failed to write to 0x1602 - 0x22F4 54"); // R2[54]
      verifyWorkingCounter(writeSDO(0x1602, 7, computePdoMapValue(0x22F4, 13, 64)), "failed to write to 0x1602 - 0x22F4 13"); // R2[13]

      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 7), "failed to write to 0x1601 - 0x8"); // num elements in 0x1602 (max 8)
   }

   public class TPDO_1a00 extends TxPDO
   {
      public TPDO_1a00()
      {
         super(0x1A00);
      }

      Signed32 measuredMotorPosition = new Signed32(); // raw motor position (0x6064)
      Float64 measuredOutputPosition = new Float64(); // raw auxiliary position, reads output encoder (0x2FE4 2)
      Float32 measuredMotorVelocity = new Float32(); // raw motor velocity (0x606C)
      Float32 measuredOutputVelocity = new Float32(); // raw output velocity (0x2FE8 2)
      Signed32 measuredBusVoltage = new Signed32(); // Bus voltage in mv (0x6079)
      Signed16 measuredMotorCurrent = new Signed16(); // motor current (0x6078)
      Float64 measuredStatorTemperature = new Float64(); // raw stator temperature from analog input channel 2 R2[19] (0x22F4 19)
   }

   private void configurePDO1a00()
   {
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 0), "failed to write to 0x1A00 -- 0x0"); // disable 0x1A00 while we write

      verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(0x6064, 0, 32)), "failed to write to 0x1A00 -- 0x6064"); // motor position
      verifyWorkingCounter(writeSDO(0x1A00, 2, computePdoMapValue(0x2FE4, 2, 64)), "failed to write to 0x1A00 -- 0x2FE4"); // output position
      verifyWorkingCounter(writeSDO(0x1A00, 3, computePdoMapValue(0x606C, 0, 32)), "failed to write to 0x1A01 -- 0x606C"); // motor velocity
      verifyWorkingCounter(writeSDO(0x1A00, 4, computePdoMapValue(0x2FE8, 2, 32)), "failed to write to 0x1A01 -- 0x2FE8"); // output velocity
      verifyWorkingCounter(writeSDO(0x1A00, 5, computePdoMapValue(0x6079, 0, 32)), "failed to write to 0x1A00 -- 0x6079"); // bus voltage
      verifyWorkingCounter(writeSDO(0x1A00, 6, computePdoMapValue(0x6078, 0, 16)), "failed to write to 0x1A00 -- 0x6078"); // torque actual
      verifyWorkingCounter(writeSDO(0x1A00, 7, computePdoMapValue(0x22F4, 19, 64)), "failed to write to 0x1A00 -- 0x22F4"); // From SIL - Getting Analog Input 2 value for stator temperature

      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 7), "failed to write to 0x1A00 -- 0x9"); // num elements in 0x1A00 (max 8)
   }

   public class TPDO_1a01 extends TxPDO
   {
      public TPDO_1a01()
      {
         super(0x1A01);
      }

      Float64 sil_desiredDahlFrictionCompensationCurrent = new Float64(); // R2[30]
      Float64 sil_desiredLinearDampingCompensationCurrent = new Float64(); // R2[31]
      Float64 sil_desiredCoggingCompensationCurrent = new Float64(); // R2[32]
      Float64 sil_desiredPDControlFeedbackCurrent = new Float64(); // R2[33]
      Float64 sil_desiredFeedForwardCurrent = new Float64(); // R2[34]
      Float64 sil_desiredTotalCurrent = new Float64(); // R2[35]
   }

   private void configurePDO1a01()
   {
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 0), "failed to write to 0x1A01 -- 0x0"); // disable 0x1A01 while we write

      verifyWorkingCounter(writeSDO(0x1A01, 1, computePdoMapValue(0x22F4, 30, 64)), "failed to write to  0x1A01 -- 0x22F4"); // R2[30] SIL dahl Friction Compensation Output Torque
      verifyWorkingCounter(writeSDO(0x1A01, 2, computePdoMapValue(0x22F4, 31, 64)), "failed to write to   0x1A01 -- 0x22F4"); // R2[31] SIL linear Damping Compensation Output Torque
      verifyWorkingCounter(writeSDO(0x1A01, 3, computePdoMapValue(0x22F4, 32, 64)), "failed to write to  0x1A01 -- 0x22F4"); // R2[32] SIL Cogging Compensation Motor Current
      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(0x22F4, 33, 64)), "failed to write to  0x1A01 -- 0x22F4"); // R2[33] SIL Acceleration Integration Motor Feedback Current
      verifyWorkingCounter(writeSDO(0x1A01, 5, computePdoMapValue(0x22F4, 34, 64)), "failed to write to  0x1A01 -- 0x22F4"); // R2[34] SIL Acceleration Integration Measured Motor Position
      verifyWorkingCounter(writeSDO(0x1A01, 6, computePdoMapValue(0x22F4, 35, 64)), "failed to write to 0x1A01  -- 0x22F4"); // R2[35] SIL Acceleration Integration Measured Motor Velocity

      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 6), "failed to write to 0x1A01 -- 0x8"); // num elements in 0x1A01 (max 8)
   }

   public class TPDO_1a02 extends TxPDO
   {
      public TPDO_1a02()
      {
         super(0x1A02);
      }

      Unsigned16 statusWord = new Unsigned16(); // Current Twitter Status SW (0x6041)
      Signed8 modeOfOperation = new Signed8(); // Current Mode of Operation (0x6061)
      Float64 sil_Socket1Warning = new Float64();
      Float64 sil_Socket1Error = new Float64();
      Float64 sil_Socket2Warning = new Float64();
      Float64 sil_Socket2Error = new Float64();
      Unsigned16 errorRegister = new Unsigned16();
      Unsigned32 statusRegister = new Unsigned32(); // Status register SR[1] (0x3607 1)
   }

   private void configurePDO1a02()
   {
      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 0), "failed to write to 0x1A02 -- 0x0"); // disable 0x1A01 while we write

      verifyWorkingCounter(writeSDO(0x1A02, 1, computePdoMapValue(0x6041, 0, 16)), "failed to write to 0x1A02 -- 0x6041"); // status word
      verifyWorkingCounter(writeSDO(0x1A02, 2, computePdoMapValue(0x6061, 0, 8)), "failed to write to 0x1A02 -- 0x6061"); // mode of operation display
      verifyWorkingCounter(writeSDO(0x1A02, 3, computePdoMapValue(0x22F4, 61, 64)), "failed to write to  0x1A02 -- 0x22F4[61]"); // Socket 1 Warning
      verifyWorkingCounter(writeSDO(0x1A02, 4, computePdoMapValue(0x22F4, 62, 64)), "failed to write to  0x1A02 -- 0x22F4[63]"); // Socket 1 Error
      verifyWorkingCounter(writeSDO(0x1A02, 5, computePdoMapValue(0x22F4, 63, 64)), "failed to write to  0x1A02 -- 0x22F4[62]"); // Socket 2 Warning
      verifyWorkingCounter(writeSDO(0x1A02, 6, computePdoMapValue(0x22F4, 64, 64)), "failed to write to  0x1A02 -- 0x22F4[64]"); // Socket 2 Error
      verifyWorkingCounter(writeSDO(0x1A02, 7, computePdoMapValue(0x603F, 0, 16)), "failed to write to 0x1A02 -- 0x603F"); // error code
      verifyWorkingCounter(writeSDO(0x1A02, 8, computePdoMapValue(0x3607, 1, 32)), "failed to write to 0x1A02 -- 0x3607"); // status register

      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 8), "failed to write to 0x1A02 -- 0x8"); // num elements in 0x1A01 (max 8)
   }

   public CycloidPlatinumTwitter(int alias, int ringPosition)
   {
      this(alias, ringPosition, TWITTER_PRODUCT_CODE.LATEST);
   }

   /**
    * @param alias EtherCAT alias of the twitter
    * @param ringPosition Position of the twitter within alias
    * @param productCode product code of the twitter
    */
   public CycloidPlatinumTwitter(int alias, int ringPosition, TWITTER_PRODUCT_CODE productCode)
   {
      super(alias, ringPosition, productCode);

      //Sync Manager, only SM2 and SM3 are used for PDOs.  SM0 and SM1 are used for the mailbox mechanism (See: EtherCAT_Application_Manual.pdf)
      SyncManager syncManager2 = new SyncManager(2, false);
      syncManager2.registerPDO(rpdo_1600);
      syncManager2.registerPDO(rpdo_1601);
      syncManager2.registerPDO(rpdo_1602);

      SyncManager syncManager3 = new SyncManager(3, false);
      syncManager3.registerPDO(tpdo_1a00);
      syncManager3.registerPDO(tpdo_1a01);
      syncManager3.registerPDO(tpdo_1a02);

      registerSyncManager(syncManager2);
      registerSyncManager(syncManager3);
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      configurePDOs();

      maxDriveCurrentMilliAmps = readSDOUnsignedInt(MAX_CURRENT_ADDRESS, 0x0);
      maxDriveCurrentAmps = maxDriveCurrentMilliAmps * AMPS_PER_MILLIAMPS;

      super.configure(dcEnabled, cycleTimeInNs);
   }

   @Override
   public boolean isResponding()
   {
      return super.isOperational();
   }

   @Override
   public Slave.State getState()
   {
      return super.getState();
   }

   //untested
   private void setFIRWindow()
   {
      verifyWorkingCounter(writeSDO(0x3289, 3, (byte) 8), "failed to write to 0x3289 3");
      verifyWorkingCounter(writeSDO(0x328A, 3, (byte) 8), "failed to write to 0x328A 3");
   }

   //untested, same as pressing save in EAS
   private void save()
   {
      verifyWorkingCounter(writeSDOASCII(0x1010, 1, "save"), "failed to write to 0x1010 1");
   }

   private void configurePDOs()
   {
      // Configure RPDOs
      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 0), "failed to write to 0x1C12");

      //1600 PDO
      configurePDO1600();
      configurePDO1601();
      configurePDO1602();

      verifyWorkingCounter(writeSDO(0x1C12, 1, (short) 0x1600), "failed to write to 0x1C12 -- 0x1600");
      verifyWorkingCounter(writeSDO(0x1C12, 2, (short) 0x1601), "failed to write to 0x1C12 -- 0x1601");
      verifyWorkingCounter(writeSDO(0x1C12, 3, (short) 0x1602), "failed to write to 0x1C12 -- 0x1602");

      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 3), "failed to write to 0x1C12  -- 0x3");

      // Configure TPDOS
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 0), "failed to write to 0x1C13");

      configurePDO1a00();
      configurePDO1a01();
      configurePDO1a02();

      verifyWorkingCounter(writeSDO(0x1C13, 1, (short) 0x1A00), "failed to write to 0x1C13 -- 0x1A00");
      verifyWorkingCounter(writeSDO(0x1C13, 2, (short) 0x1A01), "failed to write to 0x1C13 -- 0x1A01");
      verifyWorkingCounter(writeSDO(0x1C13, 3, (short) 0x1A02), "failed to write to 0x1C13 -- 0x1A02");

      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 3), "failed to write to 0x1C13 -- 0x3");

      //      writeSDO(0x1010, 1, (byte) 4);

      //set the digital outputs
      //      byte digitalOutputs = 0x0;
      //      writeSDO(0x60FE, 1, digitalOutputs);
   }

   @Override
   public void doStateControl()
   {
      super.doStateControl();
   }

   private void verifyWorkingCounter(int success, String msg)
   {
      if (success != 1)
         System.out.println(success + "," + msg);
   }

   private static int computePdoMapValue(int index, int subindex, int bitLength)
   {
      return ((index & 0xFFFF) << 16) + ((subindex & 0xFF) << 8) + (bitLength & 0xFF);
   }

   public double getAmpsPerCurrentCount()
   {
      return ampsPerCurrentCount;
   }

   public double getCurrentCountPerAmp()
   {
      return currentCountPerAmp;
   }

   public long getMaxDriveCurrentMilliAmps()
   {
      return maxDriveCurrentMilliAmps;
   }

   public double getMaxDriveCurrentAmps()
   {
      return maxDriveCurrentAmps;
   }

   @Override
   public int getElmoStatusRegister()
   {
      return (int) tpdo_1a02.statusRegister.get();
   }

   public int getErrorRegister()
   {
      return tpdo_1a02.errorRegister.get();
   }

   @Override
   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo_1a02.statusWord;
   }

   @Override
   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo_1600.controlWord;
   }

   public void setModeOfOperation(ElmoModeOfOperation enumValue)
   {
      rpdo_1600.modeOfOperation.set(enumValue.getMode());
   }

   public int getModeOfOperation()
   {
      return tpdo_1a02.modeOfOperation.get();
   }

   public double getDCLinkVoltageMilliVolts()
   {
      return tpdo_1a00.measuredBusVoltage.get();
   }

   public int getRawMotorPosition()
   {
      return tpdo_1a00.measuredMotorPosition.get();
   }

   public int getRawMeasuredCurrent()
   {
      return tpdo_1a00.measuredMotorCurrent.get();
   }

   public double getAnalogInput1a00()
   {
      return tpdo_1a00.measuredStatorTemperature.get();
   }

   public void setRawTargetPosition(int position)
   {
      rpdo_1600.targetPosition.set(position);
   }

   public void setRawTargetVelocity(int velocity)
   {
      rpdo_1600.targetVelocity.set(velocity);
   }

   /**
    * This value represents a percentage of maximum effort (torque/current). For example if this value
    * is 625, it represents a motor current of 62.5% of the maximum rated motor current (found at
    * address 0x6075).
    */
   public void setPercentageMaxEffort(int percentageMaxEffort)
   {
      rpdo_1600.targetTorquePercentage.set((short) percentageMaxEffort);
   }

   public double getRawOutputPosition()
   {
      return tpdo_1a00.measuredOutputPosition.get();
   }

   public double getRawMotorVelocity()
   {
      return tpdo_1a00.measuredMotorVelocity.get();
   }

   public double getRawOutputVelocity()
   {
      return tpdo_1a00.measuredOutputVelocity.get();
   }

   public void setR2NVMSaveFlag(boolean value)
   {
      if (value)
      {
         rpdo_1601.saveR2ToNVM.set(1);
      }
      else
      {
         rpdo_1601.saveR2ToNVM.set(0);
      }
   }

   public void setDahlFrictionForce(double dahlFrictionForce)
   {
      rpdo_1601.dahlFrictionForce.set(dahlFrictionForce);
   }

   public void setDahlSlope(double dahlSlope)
   {
      rpdo_1601.dahlSlope.set(dahlSlope);
   }

   public void setLinearDampingCompensation(double linearDampingCompensation)
   {
      rpdo_1601.linearDampingCompensation.set(linearDampingCompensation);
   }

   public void setDahlOutputScalar(double dahlOutputScalar)
   {
      rpdo_1601.dahlOutputScalar.set(dahlOutputScalar);
   }

   public void setLinearDampingOutputScalar(double linearDampingOutputScalar)
   {
      rpdo_1601.linearDampingOutputScalar.set(linearDampingOutputScalar);
   }

   public void setCoggingOutputScalar(double coggingOutputScalar)
   {
      rpdo_1601.coggingCompensationScalar.set(coggingOutputScalar);
   }

   public void setMotorDesiredPosition(double motorDesiredPositionInRadians)
   {
      rpdo_1602.desiredMotorPositionInRadians.set(motorDesiredPositionInRadians);
   }

   public void setMotorDesiredVelocity(double motorDesiredVelocityInRadians)
   {
      rpdo_1602.desiredMotorVelocityInRadians.set(motorDesiredVelocityInRadians);
   }

   public void setMaxMotorPositionError(double maxPositionError)
   {
      rpdo_1602.maxMotorFeedbackPositionError.set(maxPositionError);
   }

   public void setMaxMotorVelocityError(double maxVelocityError)
   {
      rpdo_1602.maxMotorFeedbackVelocityError.set(maxVelocityError);
   }

   public void setMotorControlStiffness(double motorControlStiffness)
   {
      rpdo_1602.motorStiffness.set(motorControlStiffness);
   }

   public void setMotorControlDamping(double motorControlDamping)
   {
      rpdo_1602.motorDamping.set(motorControlDamping);
   }

   public void setAccelerationIntegrationScalar(double accelerationIntegrationScalar)
   {
      rpdo_1602.accelerationIntegrationScalar.set(accelerationIntegrationScalar);
   }

   public double getSILDesiredFeedForwardCurrent()
   {
      return tpdo_1a01.sil_desiredFeedForwardCurrent.get();
   }

   public double getSILDesiredTotalCurrent()
   {
      return tpdo_1a01.sil_desiredTotalCurrent.get();
   }

   public double getSILDahlFrictionCompensationCurrent()
   {
      return tpdo_1a01.sil_desiredDahlFrictionCompensationCurrent.get();
   }

   public double getSILLinearDampingCompensationCurrent()
   {
      return tpdo_1a01.sil_desiredLinearDampingCompensationCurrent.get();
   }

   public double getSILDesiredPDControlFeedbackCurrent()
   {
      return tpdo_1a01.sil_desiredPDControlFeedbackCurrent.get();
   }

   public double getSILDesiredCoggingCompensationCurrent()
   {
      return tpdo_1a01.sil_desiredCoggingCompensationCurrent.get();
   }

   public double getSocket1Warning()
   {
      return tpdo_1a02.sil_Socket1Warning.get();
   }

   public double getSocket1Error()
   {
      return tpdo_1a02.sil_Socket1Error.get();
   }

   public double getSocket2Warning()
   {
      return tpdo_1a02.sil_Socket2Warning.get();
   }

   public double getSocket2Error()
   {
      return tpdo_1a02.sil_Socket2Error.get();
   }

   //TODO Implement when ready
   //   public double getMeasuredAnalogInput1a02()
   //   {
   //      return tpdo_1a02.measuredAnalogInput.get();
   //   }
}