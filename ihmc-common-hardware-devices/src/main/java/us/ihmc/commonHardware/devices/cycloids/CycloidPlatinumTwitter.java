package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.commonHardware.devices.etherCATDevices.elmo.PlatinumTwitter;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import us.ihmc.etherCAT.slaves.elmo.ElmoModeOfOperation;
import us.ihmc.hardwareStatusUI.controllerSide.ElmoTwitterDeviceStatusProvider;

import java.io.IOException;

public class CycloidPlatinumTwitter extends PlatinumTwitter implements ElmoTwitterDeviceStatusProvider
{
   private static final int MAX_CURRENT_ADDRESS = 0x6075;
   private static final int MAX_TORQUE_ADDRESS = 0x6076;

   private static final int R1 = 0x22F3;
   private static final int R2 = 0x22F4;

   private final RPDO_1600 rpdo_1600 = new RPDO_1600();
   private final RPDO_1601 rpdo_1601 = new RPDO_1601();
   private final RPDO_1602 rpdo_1602 = new RPDO_1602();
   private final TPDO_1a00 tpdo_1a00 = new TPDO_1a00();
   private final TPDO_1a01 tpdo_1a01 = new TPDO_1a01();
   private final TPDO_1a02 tpdo_1a02 = new TPDO_1a02();
   private final TPDO_1a03 tpdo_1a03 = new TPDO_1a03();

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
      Signed32 targetPosition = new Signed32(); //0x607A
      Signed32 targetVelocity = new Signed32(); //0x60FF
      Signed16 targetTorquePercentage = new Signed16(); //0x6071
   }

   private void configure1600()
   {
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write
      verifyWorkingCounter(writeSDO(0x1600, 1, computePdoMapValue(0x6040, 0, 16)), "failed to write to 0x1600 - 0x6040"); // control word
      verifyWorkingCounter(writeSDO(0x1600, 2, computePdoMapValue(0x6060, 0, 8)), "failed to write to 0x1600 - 0x6060"); // mode of operation
      verifyWorkingCounter(writeSDO(0x1600, 3, computePdoMapValue(0x607A, 0, 32)), "failed to write to 0x1600 - 0x607A"); // target position
      verifyWorkingCounter(writeSDO(0x1600, 4, computePdoMapValue(0x60FF, 0, 32)), "failed to write to 0x1600 - 0x60FF"); // target velocity
      verifyWorkingCounter(writeSDO(0x1600, 5, computePdoMapValue(0x6071, 0, 16)), "failed to write to 0x1600 - 0x6071"); // torque demand
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 5), "failed to write to 0x1600 - 0x5"); // num elements in 0x1600 (max 8)
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

      Float64 dahlFrictionCurrent = new Float64(); // Dahl Friction Current R2[1]
      Float64 dahlSlope = new Float64(); // Dahl Slope (offset by + 1.0) R2[2]
      Float64 linearDampingCompensationGain = new Float64(); // Linear Damping Compensation Gain R2[3]
      Float64 coggingOutputScalar = new Float64(); // Cogging Output Scalar R2[4]
      Float64 desiredMotorPositionInRadians = new Float64(); // Controller set desired Motor position R2[5]
      Float64 desiredMotorVelocityInRadians = new Float64(); // Controller set desired Motor velocity R2[6]
   }

   private void configure1601()
   {
      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 0), "failed to write to 0x1601 - 0x0"); // disable 0x1601 while we write
      verifyWorkingCounter(writeSDO(0x1601, 1, computePdoMapValue(R2, 1, 64)), "failed to write to 0x1601 - 0x22F4 1"); //  R2[1]
      verifyWorkingCounter(writeSDO(0x1601, 2, computePdoMapValue(R2, 2, 64)), "failed to write to 0x1601 - 0x22F4 2"); // R2[2]
      verifyWorkingCounter(writeSDO(0x1601, 3, computePdoMapValue(R2, 3, 64)), "failed to write to 0x1601 - 0x22F4 3"); // R2[3]
      verifyWorkingCounter(writeSDO(0x1601, 4, computePdoMapValue(R2, 4, 64)), "failed to write to 0x1601 - 0x22F4 4"); // R2[4]
      verifyWorkingCounter(writeSDO(0x1601, 5, computePdoMapValue(R2, 5, 64)), "failed to write to 0x1601 - 0x22F4 5"); // R2[5]
      verifyWorkingCounter(writeSDO(0x1601, 6, computePdoMapValue(R2, 6, 64)), "failed to write to 0x1601 - 0x22F4 6"); // R2[6]
      verifyWorkingCounter(writeSDO(0x1601, 0, (byte) 6), "failed to write to 0x1601 - 0x6"); // num elements in 0x1601 (max 8)
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

      Float64 maxPositionFeedbackError = new Float64(); // Max error to be used in position feedback R2[7]
      Float64 maxVelocityFeedbackError = new Float64(); // Max error to be used in velocity feedback R2[8]
      Float64 desiredMotorStiffness = new Float64(); // Impedance Control Stiffness R2[9]
      Float64 desiredMotorDamping = new Float64(); // Impedance Control Damping R2[10]
      Float64 motorPositionBreakFrequency = new Float64(); // Motor Position Alpha Filter Frequency R2[11]
      Float64 outputPositionBreakFrequency = new Float64(); // Output Position Alpha Filter Frequency R2[12]
      Float64 motorVelocityBreakFrequency = new Float64(); // Motor Velocity Alpha Filter Frequency R2[13]
      Float64 outputVelocityBreakFrequency = new Float64(); // Output Velocity Alpha Filter Frequency R2[14]
   }

   private void configure1602()
   {
      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 0), "failed to write to 0x1602 - 0x0"); // disable 0x1601 while we write
      verifyWorkingCounter(writeSDO(0x1602, 1, computePdoMapValue(R2, 7, 64)), "failed to write to 0x1602 - 0x22F4 7"); // R2[7]
      verifyWorkingCounter(writeSDO(0x1602, 2, computePdoMapValue(R2, 8, 64)), "failed to write to 0x1602 - 0x22F4 8"); // R2[8]
      verifyWorkingCounter(writeSDO(0x1602, 3, computePdoMapValue(R2, 9, 64)), "failed to write to 0x1602 - 0x22F4 9"); // R2[9]
      verifyWorkingCounter(writeSDO(0x1602, 4, computePdoMapValue(R2, 10, 64)), "failed to write to 0x1602 - 0x22F4 10"); // R2[10]
      verifyWorkingCounter(writeSDO(0x1602, 5, computePdoMapValue(R2, 11, 64)), "failed to write to 0x1602 - 0x22F4 11"); // R2[11]
      verifyWorkingCounter(writeSDO(0x1602, 6, computePdoMapValue(R2, 12, 64)), "failed to write to 0x1602 - 0x22F4 12"); // R2[12]
      verifyWorkingCounter(writeSDO(0x1602, 7, computePdoMapValue(R2, 13, 64)), "failed to write to 0x1602 - 0x22F4 13"); // R2[13]
      verifyWorkingCounter(writeSDO(0x1602, 8, computePdoMapValue(R2, 14, 64)), "failed to write to 0x1602 - 0x22F4 14"); // R2[14]
      verifyWorkingCounter(writeSDO(0x1602, 0, (byte) 8), "failed to write to 0x1602 - 0x8"); // num elements in 0x1602 (max 8)
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

      Unsigned16 statusWord = new Unsigned16(); // 0x6041
      Signed8 modeOfOperation = new Signed8(); // 0x6061
      Signed32 measuredBusVoltage = new Signed32(); // 0x6079
      Signed16 measuredCurrent = new Signed16(); // 0x6078
      Unsigned32 statusRegister = new Unsigned32(); // 0x3607 1
      Unsigned16 errorRegister = new Unsigned16(); // 0x603F
   }

   private void configure1A00()
   {
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 0), "failed to write to 0x1A00 -- 0x0"); // disable 0x1A00 while we write
      verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(0x6041, 0, 16)), "failed to write to 0x1A00 -- 0x6041"); // status word
      verifyWorkingCounter(writeSDO(0x1A00, 2, computePdoMapValue(0x6061, 0, 8)), "failed to write to 0x1A00 -- 0x6061"); // mode of operation display
      verifyWorkingCounter(writeSDO(0x1A00, 3, computePdoMapValue(0x6079, 0, 32)), "failed to write to 0x1A00 -- 0x6079"); // bus voltage
      verifyWorkingCounter(writeSDO(0x1A00, 4, computePdoMapValue(0x6078, 0, 16)), "failed to write to 0x1A00 -- 0x6078"); // current actual
      verifyWorkingCounter(writeSDO(0x1A00, 5, computePdoMapValue(0x3607, 1, 32)), "failed to write to 0x1A00 -- 0x3607"); // status register
      verifyWorkingCounter(writeSDO(0x1A00, 6, computePdoMapValue(0x603F, 0, 16)), "failed to write to 0x1A02 -- 0x603F"); // error register
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 6), "failed to write to 0x1A00 -- 0x9"); // num elements in 0x1A00 (max 8)
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

      Float64 sil_desiredDahlFrictionCompensationCurrent = new Float64(); // R2[31]
      Float64 sil_desiredLinearDampingCompensationCurrent = new Float64(); // R2[32]
      Float64 sil_desiredCoggingCompensationCurrent = new Float64(); // R2[33]
      Float64 sil_desiredPDControlFeedbackCurrent = new Float64(); // R2[34]

      // This term should be equal to the current that is sent down on 0x6071, but binned based on the resolution, and
      // scaled based on the soft start
      Float64 sil_desiredFeedForwardCurrent = new Float64(); // R2[35]
      // This is the total current, which is the feedforward + compensation + feedback
      Float64 sil_desiredTotalCurrent = new Float64(); // R2[36]
   }

   private void configure1A01()
   {
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 0), "failed to write to 0x1A01 -- 0x0"); // disable 0x1A01 while we write
      verifyWorkingCounter(writeSDO(0x1A01, 1, computePdoMapValue(R2, 31, 64)), "failed to write to 0x1A01 -- 0X22F4 31"); // R2[31]
      verifyWorkingCounter(writeSDO(0x1A01, 2, computePdoMapValue(R2, 32, 64)), "failed to write to 0x1A01 -- 0x22F4 32"); // R2[32]
      verifyWorkingCounter(writeSDO(0x1A01, 3, computePdoMapValue(R2, 33, 64)), "failed to write to 0x1A01 -- 0x22F4 33"); // R2[33]
      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(R2, 34, 64)), "failed to write to 0x1A01 -- 0x22F4 34"); // R2[34]
      verifyWorkingCounter(writeSDO(0x1A01, 5, computePdoMapValue(R2, 35, 64)), "failed to write to 0x1A01 -- 0x22F4 35"); // R2[35]
      verifyWorkingCounter(writeSDO(0x1A01, 6, computePdoMapValue(R2, 36, 64)), "failed to write to 0x1A01 -- 0x22F4 36"); // R2[36]
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 6), "failed to write to 0x1A01 -- 0x8"); // num elements in 0x1A01 (max 8)
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

      //      Float64 controlDT = new Float64(); // R2[41]
      Signed32 sil_Socket1Warning = new Signed32(); // R1[11]
      Signed32 sil_Socket1Error = new Signed32(); // R1[12]
      Signed32 sil_Socket2Warning = new Signed32(); // R1[13]
      Signed32 sil_Socket2Error = new Signed32(); // R1[14]
      Signed32 sil_AnalogInput1 = new Signed32(); // R1[15] TODO: Temp sensor??
      Signed32 sil_AnalogInput2 = new Signed32(); // R1[16]
   }

   private void configure1A02()
   {
      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 0), "failed to write to 0x1A02 -- 0x0"); // disable 0x1A01 while we write
      verifyWorkingCounter(writeSDO(0x1A02, 1, computePdoMapValue(R1, 11, 32)), "failed to write to  0x1A02 -- 0x22F3 11"); // R1[11] Socket 1 Warning
      verifyWorkingCounter(writeSDO(0x1A02, 2, computePdoMapValue(R1, 12, 32)), "failed to write to  0x1A02 -- 0x22F3 12"); // R1[12] Socket 1 Error
      verifyWorkingCounter(writeSDO(0x1A02, 3, computePdoMapValue(R1, 13, 32)), "failed to write to  0x1A02 -- 0x22F3 13"); // R1[13] Socket 2 Warning
      verifyWorkingCounter(writeSDO(0x1A02, 4, computePdoMapValue(R1, 14, 32)), "failed to write to  0x1A02 -- 0x22F3 14"); // R1[14] Socket 2 Error
      verifyWorkingCounter(writeSDO(0x1A02, 5, computePdoMapValue(R1, 15, 32)), "failed to write to  0x1A02 -- 0x22F3 15"); // R1[15] Analog 1 Input TODO:
      verifyWorkingCounter(writeSDO(0x1A02, 6, computePdoMapValue(R1, 16, 32)),
                           "failed to write to  0x1A02 -- 0x22F3 16"); // R1[16] From SIL - Getting Analog Input 2 value for stator temperature
      verifyWorkingCounter(writeSDO(0x1A02, 0, (byte) 6), "failed to write to 0x1A02 -- 0x6"); // num elements in 0x1A02 (max 8)
   }

   public class TPDO_1a03 extends TxPDO
   {
      public TPDO_1a03()
      {
         super(0x1A03);
      }

      Float64 measuredMotorPosition = new Float64(); // R2[41] Filtered. R2[37] Unfiltered
      Float64 measuredOutputPosition = new Float64(); // R2[42] Filtered. R2[38] Unfiltered
      Float64 measuredMotorVelocity = new Float64(); // R2[43] Filtered. R2[39] Unfiltered
      Float64 measuredOutputVelocity = new Float64(); // R2[44] Filtered. R2[40] Unfiltered
      Float64 controlTime = new Float64(); // R2[45]
      Float64 silTemp = new Float64(); // R2[46]
   }

   private void configure1A03()
   {
      verifyWorkingCounter(writeSDO(0x1A03, 0, (byte) 0), "failed to write to 0x1A03 -- 0x0"); // disable 0x1A03 while we write
      verifyWorkingCounter(writeSDO(0x1A03, 1, computePdoMapValue(R2, 41, 64)), "failed to write to 0x1A03 -- 0x22F4 41"); // R2[41]
      verifyWorkingCounter(writeSDO(0x1A03, 2, computePdoMapValue(R2, 42, 64)), "failed to write to 0x1A03 -- 0x22F4 42"); // R2[42]
      verifyWorkingCounter(writeSDO(0x1A03, 3, computePdoMapValue(R2, 43, 64)), "failed to write to 0x1A03 -- 0x22F4 43"); // R2[43]
      verifyWorkingCounter(writeSDO(0x1A03, 4, computePdoMapValue(R2, 44, 64)), "failed to write to 0x1A03 -- 0x22F4 44"); // R2[44]
      verifyWorkingCounter(writeSDO(0x1A03, 5, computePdoMapValue(R2, 45, 64)), "failed to write to 0x1A03 -- 0x22F4 45"); // R2[45]
      verifyWorkingCounter(writeSDO(0x1A03, 6, computePdoMapValue(R2, 46, 64)), "failed to write to 0x1A03 -- 0x22F4 46"); // R2[46]
      verifyWorkingCounter(writeSDO(0x1A03, 0, (byte) 6), "failed to write to 0x1A03 -- 0x6"); // num elements in 0x1A03 (max 8)
   }

   public CycloidPlatinumTwitter(int alias, int ringPosition)
   {
      this(alias, ringPosition, TWITTER_PRODUCT_CODE.LATEST);
   }

   /**
    * @param alias
    * @param ringPosition
    * @throws IOException
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
      syncManager3.registerPDO(tpdo_1a03);

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

   /**
    * Configure all the registries on master to correspond with the correct registries on the twitter for their specific action
    */
   private void configurePDOs()
   {
      // Configure RPDOs
      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 0), "failed to write to 0x1C12");

      //1600 PDO
      configure1600();

      //1601 PDO
      configure1601();

      //1602 PDO
      configure1602();

      verifyWorkingCounter(writeSDO(0x1C12, 1, (short) 0x1600), "failed to write to 0x1C12 -- 0x1600");
      verifyWorkingCounter(writeSDO(0x1C12, 2, (short) 0x1601), "failed to write to 0x1C12 -- 0x1601");
      verifyWorkingCounter(writeSDO(0x1C12, 3, (short) 0x1602), "failed to write to 0x1C12 -- 0x1602");

      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 3), "failed to write to 0x1C12  -- 0x2");

      // Configure TPDOS
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 0), "failed to write to 0x1C13");

      // tpdo_1a00
      configure1A00();

      // tpdo_1a01
      configure1A01();

      // tpdo_1a02
      configure1A02();

      // tpdo_1a03
      configure1A03();

      verifyWorkingCounter(writeSDO(0x1C13, 1, (short) 0x1A00), "failed to write to 0x1C13 -- 0x1A00");
      verifyWorkingCounter(writeSDO(0x1C13, 2, (short) 0x1A01), "failed to write to 0x1C13 -- 0x1A01");
      verifyWorkingCounter(writeSDO(0x1C13, 3, (short) 0x1A02), "failed to write to 0x1C13 -- 0x1A02");
      verifyWorkingCounter(writeSDO(0x1C13, 4, (short) 0x1A03), "failed to write to 0x1C13 -- 0x1A03");
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 4), "failed to write to 0x1C13 -- 0x4");
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

   @Override
   public boolean isFaulted()
   {
      return super.isFaulted();
   }

   @Override
   public boolean isUnderVoltage()
   {
      return super.isUnderVoltage();
   }

   @Override
   public boolean isOverVoltage()
   {
      return super.isOverVoltage();
   }

   @Override
   public boolean isSTODisabled()
   {
      return super.isSTODisabled();
   }

   @Override
   public boolean isCurrentShort()
   {
      return super.isCurrentShorted();
   }

   @Override
   public boolean isOverTemp()
   {
      return super.isOverTemperature();
   }

   @Override
   public int getElmoErrorCode()
   {
      return getErrorRegister();
   }

   @Override
   public double getInputEncoderError()
   {
      return getSocket1Error();
   }

   @Override
   public double getOutputEncoderError()
   {
      return getSocket2Error();
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
      return tpdo_1a00.errorRegister.get();
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

   public int getRawMeasuredCurrent()
   {
      return tpdo_1a00.measuredCurrent.get();
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

   public void setDahlFrictionForce(double dahlFrictionForce)
   {
      rpdo_1601.dahlFrictionCurrent.set(dahlFrictionForce);
   }

   public void setDahlSlope(double dahlSlope)
   {
      rpdo_1601.dahlSlope.set(dahlSlope);
   }

   public void setLinearDampingCompensation(double linearDampingCompensation)
   {
      rpdo_1601.linearDampingCompensationGain.set(linearDampingCompensation);
   }

   public void setCoggingOutputScalar(double coggingOutputScalar)
   {
      rpdo_1601.coggingOutputScalar.set(coggingOutputScalar);
   }

   public void setDesiredMotorPositionForImpedanceControl(double motorDesiredPositionInRadians)
   {
      rpdo_1601.desiredMotorPositionInRadians.set(motorDesiredPositionInRadians);
   }

   public void setDesiredMotorVelocityForImpedanceControl(double motorDesiredVelocityInRadians)
   {
      rpdo_1601.desiredMotorVelocityInRadians.set(motorDesiredVelocityInRadians);
   }

   public void setMaxMotorPositionError(double maxPositionError)
   {
      rpdo_1602.maxPositionFeedbackError.set(maxPositionError);
   }

   public void setMaxMotorVelocityError(double maxVelocityError)
   {
      rpdo_1602.maxVelocityFeedbackError.set(maxVelocityError);
   }

   public void setMotorStiffnessForImpedanceControl(double stiffness)
   {
      rpdo_1602.desiredMotorStiffness.set(stiffness);
   }

   public void setMotorDampingForImpedanceControl(double damping)
   {
      rpdo_1602.desiredMotorDamping.set(damping);
   }

   public void setMotorPositionBreakFrequency(double motorPositionBreakFrequency)
   {
      rpdo_1602.motorPositionBreakFrequency.set(motorPositionBreakFrequency);
   }

   public void setOutputPositionBreakFrequency(double outputPositionBreakFrequency)
   {
      rpdo_1602.outputPositionBreakFrequency.set(outputPositionBreakFrequency);
   }

   public void setMotorVelocityBreakFrequency(double motorVelocityBreakFrequency)
   {
      rpdo_1602.motorVelocityBreakFrequency.set(motorVelocityBreakFrequency);
   }

   public void setOutputVelocityBreakFrequency(double outputVelocityBreakFrequency)
   {
      rpdo_1602.outputVelocityBreakFrequency.set(outputVelocityBreakFrequency);
   }

   public double getMeasuredMotorPosition()
   {
      return tpdo_1a03.measuredMotorPosition.get();
   }

   public double getMeasuredOutputPosition()
   {
      return tpdo_1a03.measuredOutputPosition.get();
   }

   public double getMeasuredMotorVelocity()
   {
      return tpdo_1a03.measuredMotorVelocity.get();
   }

   public double getMeasuredOutputVelocity()
   {
      return tpdo_1a03.measuredOutputVelocity.get();
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

   public int getSocket1Warning()
   {
      return tpdo_1a02.sil_Socket1Warning.get();
   }

   public int getSocket1Error()
   {
      return tpdo_1a02.sil_Socket1Error.get();
   }

   public int getSocket2Warning()
   {
      return tpdo_1a02.sil_Socket2Warning.get();
   }

   public int getSocket2Error()
   {
      return tpdo_1a02.sil_Socket2Error.get();
   }

   public int getMeasuredAnalogInput1()
   {
      return tpdo_1a02.sil_AnalogInput1.get();
   }

   public int getMeasuredAnalogInput2()
   {
      return tpdo_1a02.sil_AnalogInput2.get();
   }

   public double getSILTemperature()
   {
      return tpdo_1a03.silTemp.get();
   }

   public double getSILControlTime()
   {
      return tpdo_1a03.controlTime.get();
   }
}