package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.PlatinumTwitter;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.EtherCATDeviceStatusProvider;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;

import java.io.IOException;

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

   /**
    * This class contains some of the variables commanded for the twitter, assigned to 0x1600
    * Each value is written by the master and read by the twitter
    */
   public class RPDO_1600 extends RxPDO
   {
      public RPDO_1600()
      {
         super(0x1600);
      }

      Unsigned16 controlWord = new Unsigned16(); //0x6040
      Unsigned8 modeOfOperation = new Unsigned8(); //0x6060
      Signed16 targetTorquePercentage = new Signed16(); //0x6071
   }

   /**
    * This class contains some of the variables commanded for the twitter, assigned to 0x1601
    * Each value is written by the master and read by the twitter
    */
   public class RPDO_1601 extends RxPDO
   {
      public RPDO_1601()
      {
         super(0x1601);
      }

      Signed32 saveR2ToNVM = new Signed32(); //Save R2 to NVM

      Float64 dahlFrictionForce = new Float64(); // Dahl Friction Force
      Float64 dahlSlope = new Float64(); // Dahl Slope (offset by + 1.0)
      Float64 linearDampingCompensation = new Float64(); // Linear Damping Compensation
      Float64 dahlOutputScalar = new Float64(); // Dahl Output Scalar
      Float64 linearDampingOutputScalar = new Float64(); // Linear Damping Output Scalar
      Float64 coggingOutputScalar = new Float64(); // Cogging Output Scalar
   }

   /**
    * This class contains some of the variables commanded for the twitter, assigned to 0x1602
    * Each value is written by the master and read by the twitter
    */
   public class RPDO_1602 extends RxPDO
   {
      public RPDO_1602()
      {
         super(0x1602);
      }

      Float64 desiredMotorPositionInRadians = new Float64(); // Controller set desired Motor position R2[51]
      Float64 desiredMotorVelocityInRadians = new Float64(); // Controller set desired Motor velocity R2[52]
      Float64 accelerationIntegrationStiffness = new Float64(); // Stiffness R2[55]
      Float64 accelerationIntegrationDamping = new Float64(); // Damping R2[56]
      Float64 accelerationIntegrationScalar = new Float64(); // Position and velocity FeedBack Scalar R2[13]
      Float64 accelerationIntegrationMaxPositionError = new Float64(); // R2[53]
      Float64 accelerationIntegrationMaxVelocityError = new Float64(); // R2[54]
   }

   /**
    * This class contains some of the variables received from the twitter, assigned to 0x1A00
    * Each value is written by the twitter and read by the master
    */
   public class TPDO_1a00 extends TxPDO
   {
      public TPDO_1a00()
      {
         super(0x1A00);
      }

      Unsigned16 statusWord = new Unsigned16();
      Signed8 modeOfOperation = new Signed8();
      Signed32 measuredPosition = new Signed32();
      Signed32 measuredBusVoltage = new Signed32();
      Signed16 measuredCurrent = new Signed16();
      Float64 measuredAnalogInput = new Float64();
      Float64 measuredAuxPosition = new Float64();
      Unsigned32 statusRegister = new Unsigned32();
   }

   /**
    * This class contains some of the variables received from the twitter, assigned to 0x1A01
    * Each value is written by the twitter and read by the master
    */
   public class TPDO_1a01 extends TxPDO
   {
      public TPDO_1a01()
      {
         super(0x1A01);
      }

      Float32 measuredMotorVelocity = new Float32();
      Float32 measuredOutputVelocity = new Float32();

      Float64 sil_desiredDahlFrictionCompensationCurrent = new Float64();
      Float64 sil_desiredLinearDampingCompensationCurrent = new Float64();
      Float64 sil_desiredCoggingCompensationCurrent = new Float64();
      Float64 sil_desiredPDControlFeedbackCurrent = new Float64();

      Float64 sil_desiredFeedForwardCurrent = new Float64();
      Float64 sil_desiredTotalCurrent = new Float64();
   }

   /**
    * This class contains some of the variables received from the twitter, assigned to 0x1A02
    * Each value is written by the twitter and read by the master
    */
   public class TPDO_1a02 extends TxPDO
   {
      public TPDO_1a02()
      {
         super(0x1A02);
      }

//      Float64 sil_Socket1Warning = new Float64();
//      Float64 sil_Socket1Error = new Float64();
//      Float64 sil_Socket2Warning = new Float64();
//      Float64 sil_Socket2Error = new Float64();
      Unsigned16 errorRegister = new Unsigned16();
   }

   public CycloidPlatinumTwitter(int alias, int ringPosition)
   {
      this(alias, ringPosition, TWITTER_PRODUCT_CODE.LATEST);
   }

   /**
    * Constructs the cycloid platinum twitter
    *
    * @param alias        Alias of the twitter for etherCAT
    * @param ringPosition Position of the twitter for etherCAT
    * @param productCode  Specific product code for the type of twitter
    * @throws IOException If there is an issue with initialization or registration for the read or write, an exception will be thrown
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

   /**
    * Configure all the registries on master to correspond with the correct registries on the twitter for their specific action
    */
   private void configurePDOs()
   {
      // Configure RPDOs
      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 0), "failed to write to 0x1C12");

      configureRPDO1600();
      configureRPDO1601();
      configureRPDO1602();

      verifyWorkingCounter(writeSDO(0x1C12, 1, (short) 0x1600), "failed to write to 0x1C12 -- 0x1600");
      verifyWorkingCounter(writeSDO(0x1C12, 2, (short) 0x1601), "failed to write to 0x1C12 -- 0x1601");
      verifyWorkingCounter(writeSDO(0x1C12, 3, (short) 0x1602), "failed to write to 0x1C12 -- 0x1601");

      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 3), "failed to write to 0x1C12  -- 0x2");

      // Configure TPDOS
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 0), "failed to write to 0x1C13");

      configureTPDO1A00();
      configureTPDO1A01();
      configureTPDO1A02();

      verifyWorkingCounter(writeSDO(0x1C13, 1, (short) 0x1A00), "failed to write to 0x1C13 -- 0x1A00");
      verifyWorkingCounter(writeSDO(0x1C13, 2, (short) 0x1A01), "failed to write to 0x1C13 -- 0x1A01");
      verifyWorkingCounter(writeSDO(0x1C13, 3, (short) 0x1A02), "failed to write to 0x1C13 -- 0x1A02");
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 3), "failed to write to 0x1C13 -- 0x2");
   }

   /**
    * Configure the RPDO connected to 0x1600 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureRPDO1600()
   {
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1600, 1, computePdoMapValue(0x6040, 0, 16)), "failed to write to 0x1600 - 0x6040"); // control word
      verifyWorkingCounter(writeSDO(0x1600, 2, computePdoMapValue(0x6060, 0, 8)), "failed to write to 0x1600 - 0x6060"); // mode of operation
      verifyWorkingCounter(writeSDO(0x1600, 3, computePdoMapValue(0x6071, 0, 16)), "failed to write to 0x1600 - 0x6071"); // torque demand

      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 3), "failed to write to 0x1600 - 0x3"); // num elements in 0x1600 (max 8)
   }

   /**
    * Configure the RPDO connected to 0x1601 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureRPDO1601()
   {
      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 0), "failed to write to 0x1601 - 0x0"); // disable 0x1601 while we write

      verifyWorkingCounter(writeSDO(0x1601, 1, computePdoMapValue(0x22F3, 2, 32)), "failed to write to 0x1600 - 0x22F3"); //    R1 index 2
      verifyWorkingCounter(writeSDO(0x1601, 2, computePdoMapValue(0x22F4, 1, 64)), "failed to write to 0x1600 - 0x22F4 1"); //  R2 index 1
      verifyWorkingCounter(writeSDO(0x1601, 3, computePdoMapValue(0x22F4, 2, 64)), "failed to write to 0x1600 - 0x22F4 2"); // R2 index 2
      verifyWorkingCounter(writeSDO(0x1601, 4, computePdoMapValue(0x22F4, 3, 64)), "failed to write to 0x1600 - 0x22F4 3"); // R2 index 3
      verifyWorkingCounter(writeSDO(0x1601, 5, computePdoMapValue(0x22F4, 10, 64)), "failed to write to 0x1600 - 0x22F4 10"); // R2 index 10
      verifyWorkingCounter(writeSDO(0x1601, 6, computePdoMapValue(0x22F4, 11, 64)), "failed to write to 0x1600 - 0x22F4 11"); // R2 index 11
      verifyWorkingCounter(writeSDO(0x1601, 7, computePdoMapValue(0x22F4, 12, 64)), "failed to write to 0x1600 - 0x22F4 12"); // R2 index 12

      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 7), "failed to write to 0x1601 - 0x8"); // num elements in 0x1600 (max 8)
   }

   /**
    * Configure the RPDO connected to 0x1602 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureRPDO1602()
   {
      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 0), "failed to write to 0x1601 - 0x0"); // disable 0x1601 while we write

      verifyWorkingCounter(writeSDO(0x1602, 1, computePdoMapValue(0x22F4, 51, 64)), "failed to write to 0x1600 - 0x22F4"); //   Desired Position R2[51]
      verifyWorkingCounter(writeSDO(0x1602, 2, computePdoMapValue(0x22F4, 52, 64)), "failed to write to 0x1600 - 0x22F4 1"); // Desired Velocity R2[52]
      verifyWorkingCounter(writeSDO(0x1602, 3, computePdoMapValue(0x22F4, 55, 64)), "failed to write to 0x1600 - 0x22F4 2"); // Stiffness R2[55]
      verifyWorkingCounter(writeSDO(0x1602, 4, computePdoMapValue(0x22F4, 56, 64)), "failed to write to 0x1600 - 0x22F4 3"); // Damping R2[56]
      verifyWorkingCounter(writeSDO(0x1602, 5, computePdoMapValue(0x22F4, 53, 64)), "failed to write to 0x1600 - 0x22F4"); //   Max position error R2[53]
      verifyWorkingCounter(writeSDO(0x1602, 6, computePdoMapValue(0x22F4, 54, 64)), "failed to write to 0x1600 - 0x22F4 1"); // Max velocity error R2[54]
      verifyWorkingCounter(writeSDO(0x1602, 7, computePdoMapValue(0x22F4, 13, 64)), "failed to write to 0x1600 - 0x22F4 10"); // Position and Velocity Feedback Scalar R2[13]

      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 7), "failed to write to 0x1601 - 0x8"); // num elements in 0x1602 (max 8)

   }

   /**
    * Configure the TPDO connected to 0x1A00 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureTPDO1A00()
   {
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 0), "failed to write to 0x1A00 -- 0x0"); // disable 0x1A00 while we write

      verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(0x6041, 0, 16)), "failed to write to 0x1A00 -- 0x6041"); // status word
      verifyWorkingCounter(writeSDO(0x1A00, 2, computePdoMapValue(0x6061, 0, 8)), "failed to write to 0x1A00 -- 0x6061"); // mode of operation display
      verifyWorkingCounter(writeSDO(0x1A00, 3, computePdoMapValue(0x6064, 0, 32)), "failed to write to 0x1A00 -- 0x6064"); // position actual
      verifyWorkingCounter(writeSDO(0x1A00, 4, computePdoMapValue(0x6079, 0, 32)), "failed to write to 0x1A00 -- 0x6079"); // bus voltage
      verifyWorkingCounter(writeSDO(0x1A00, 5, computePdoMapValue(0x6077, 0, 16)), "failed to write to 0x1A00 -- 0x6077"); // torque actual
      verifyWorkingCounter(writeSDO(0x1A00, 6, computePdoMapValue(0x22F4, 19, 64)), "failed to write to 0x1A00 -- 0x22F4"); // From SIL - Getting Analog Input 2 value for stator temperature
      verifyWorkingCounter(writeSDO(0x1A00, 7, computePdoMapValue(0x2FE4, 2, 64)), "failed to write to 0x1A00 -- 0x2FE4"); // aux position 1
      verifyWorkingCounter(writeSDO(0x1A00, 8, computePdoMapValue(0x3607, 1, 32)), "failed to write to 0x1A00 -- 0x3607"); // status register

      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 8), "failed to write to 0x1A00 -- 0x9"); // num elements in 0x1A00 (max 8)
   }

   /**
    * Configure the TPDO connected to 0x1A00 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureTPDO1A01()
   {
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 0), "failed to write to 0x1A01 -- 0x0"); // disable 0x1A01 while we write

      verifyWorkingCounter(writeSDO(0x1A01, 1, computePdoMapValue(0x2FE8, 1, 32)), "failed to write to 0x1A01 -- 0x2FE8"); // motor velocity
      verifyWorkingCounter(writeSDO(0x1A01, 2, computePdoMapValue(0x2FE8, 2, 32)), "failed to write to 0x1A01 -- 0x2FE8"); // output velocity
      verifyWorkingCounter(writeSDO(0x1A01, 3, computePdoMapValue(0x22F4, 30, 64)),
                           "failed to write to  0x1A01 -- 0x22F4"); // SIL dahl Friction Compensation Output Torque
      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(0x22F4, 31, 64)),
                           "failed to write to   0x1A01 -- 0x22F4"); // SIL linear Damping Compensation Output Torque
      verifyWorkingCounter(writeSDO(0x1A01, 5, computePdoMapValue(0x22F4, 32, 64)),
                           "failed to write to  0x1A01 -- 0x22F4"); // SIL Cogging Compensation Motor Current
      verifyWorkingCounter(writeSDO(0x1A01, 6, computePdoMapValue(0x22F4, 33, 64)),
                           "failed to write to  0x1A01 -- 0x22F4"); // SIL Acceleration Integration Motor Feedback Current
      verifyWorkingCounter(writeSDO(0x1A01, 7, computePdoMapValue(0x22F4, 41, 64)),
                           "failed to write to  0x1A01 -- 0x22F4"); // SIL Acceleration Integration Measured Motor Position
      verifyWorkingCounter(writeSDO(0x1A01, 8, computePdoMapValue(0x22F4, 42, 64)),
                           "failed to write to 0x1A01  -- 0x22F4"); // SIL Acceleration Integration Measured Motor Velocity

      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 8), "failed to write to 0x1A01 -- 0x8"); // num elements in 0x1A01 (max 8)
   }

   /**
    * Configure the TPDO connected to 0x1A00 with register values from the twitter. A max of 8 variables can be mapped to the PDO
    */
   private void configureTPDO1A02()
   {
      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 0), "failed to write to 0x1A02 -- 0x0"); // disable 0x1A01 while we write

//      verifyWorkingCounter(writeSDO(0x1A02, 1, computePdoMapValue(0x22F4, 61, 64)), "failed to write to  0x1A02 -- 0x22F4 index 61"); // Socket 1 Warning
//      verifyWorkingCounter(writeSDO(0x1A02, 2, computePdoMapValue(0x22F4, 62, 64)), "failed to write to  0x1A02 -- 0x22F4 index 63"); // Socket 1 Error
//      verifyWorkingCounter(writeSDO(0x1A02, 3, computePdoMapValue(0x22F4, 63, 64)), "failed to write to  0x1A02 -- 0x22F4 index 62"); // Socket 2 Warning
//      verifyWorkingCounter(writeSDO(0x1A02, 4, computePdoMapValue(0x22F4, 64, 64)), "failed to write to  0x1A02 -- 0x22F4 index 64"); // Socket 2 Error
      verifyWorkingCounter(writeSDO(0x1A02, 1, computePdoMapValue(0x603F, 0, 16)), "failed to write to 0x1A00 -- 0x603F"); // error code

      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 1), "failed to write to 0x1A02 -- 0x5"); // num elements in 0x1A01 (max 8)
   }

   @Override
   public void doStateControl()
   {
      super.doStateControl();
   }

   /**
    * Verify that the registration of the specific registry
    *
    * @param success If 1, the registration is a success, else a fail
    * @param msg     Failure message to be printed out
    */
   private void verifyWorkingCounter(int success, String msg)
   {
      if (success != 1)
         System.out.println(success + "," + msg);
   }

   /**
    * Compute the actual pdo map value for registration
    *
    * @param index     Initial index of the pdo
    * @param subindex  Index offset of the pdo
    * @param bitLength Length of the data desired
    * @return The actual pdo map value
    */
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
      return (int) tpdo_1a00.statusRegister.get();
   }

   public int getErrorRegister()
   {
      return tpdo_1a02.errorRegister.get();
   }

   @Override
   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo_1a00.statusWord;
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
      return tpdo_1a00.modeOfOperation.get();
   }

   public double getDCLinkVoltageMilliVolts()
   {
      return tpdo_1a00.measuredBusVoltage.get();
   }

   public int getRawMotorEncoderPosition()
   {
      return tpdo_1a00.measuredPosition.get();
   }

   public int getRawMeasuredCurrent()
   {
      return tpdo_1a00.measuredCurrent.get();
   }

   public double getAnalogInput1a00()
   {
      return tpdo_1a00.measuredAnalogInput.get();
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

   public double getRawAuxiliaryPosition()
   {
      return tpdo_1a00.measuredAuxPosition.get();
   }

   public double getRawMotorVelocity()
   {
      return tpdo_1a01.measuredMotorVelocity.get();
   }

   public double getRawAuxiliaryVelocity()
   {
      return tpdo_1a01.measuredOutputVelocity.get();
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
      rpdo_1601.coggingOutputScalar.set(coggingOutputScalar);
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
      rpdo_1602.accelerationIntegrationMaxPositionError.set(maxPositionError);
   }

   public void setMaxMotorVelocityError(double maxVelocityError)
   {
      rpdo_1602.accelerationIntegrationMaxVelocityError.set(maxVelocityError);
   }

   public void setAccelerationIntegrationStiffness(double accelerationIntegrationStiffness)
   {
      rpdo_1602.accelerationIntegrationStiffness.set(accelerationIntegrationStiffness);
   }

   public void setAccelerationIntegrationDamping(double accelerationIntegrationDamping)
   {
      rpdo_1602.accelerationIntegrationDamping.set(accelerationIntegrationDamping);
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

//   public double getSocket1Warning()
//   {
//      return tpdo_1a02.sil_Socket1Warning.get();
//   }
//
//   public double getSocket1Error()
//   {
//      return tpdo_1a02.sil_Socket1Error.get();
//   }
//
//   public double getSocket2Warning()
//   {
//      return tpdo_1a02.sil_Socket2Warning.get();
//   }
//
//   public double getSocket2Error()
//   {
//      return tpdo_1a02.sil_Socket2Error.get();
//   }
}