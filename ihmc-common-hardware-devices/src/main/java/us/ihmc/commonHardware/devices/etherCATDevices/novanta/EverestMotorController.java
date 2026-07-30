package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import us.ihmc.etherCAT.slaves.DSP402Slave;

import static us.ihmc.commonHardware.devices.etherCATDevices.novanta.EverestRegisters.*;

public class EverestMotorController extends DSP402Slave
{
   private static final int VENDOR_ID = 0x0000029C;
   private static final int PRODUCT_CODE = 0x03B31001;
   private float tc;
   private RPDO rpdo = new RPDO();
   private TPDO_1 tpdo_1 = new TPDO_1();
   private TPDO_2 tpdo_2 = new TPDO_2();
   private boolean maxConfig = false;
   public class RPDO extends RxPDO
   {
      public RPDO()
      {
         super(0x1600);
      }

      Unsigned16 controlWord = new Unsigned16(); //0x2010 or 0x6040
      Signed8 desiredOperationMode = new Signed8(); //0x6060 (0x2014 holds a more complex version)
      Float32 quadratureCurrentSetPoint = new Float32(); //0x201A
      Signed32 positionSetPoint = new Signed32();
      Float32 velocitySetPoint = new Float32();


   }

   private void configureRPDO()
   {
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1600, 1, computePdoMapValue(CIA_CONTROL_WORD.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1600, 2, computePdoMapValue(MODES_OF_OPERATION.getAddress(), 0, 8)), "failed to write to 0x1600 - 0x6060");
      verifyWorkingCounter(writeSDO(0x1600, 3, computePdoMapValue(CURRENT_QUADRATURE_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x201A");
      verifyWorkingCounter(writeSDO(0x1600, 4, computePdoMapValue(POSITION_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2020");
      verifyWorkingCounter(writeSDO(0x1600, 5, computePdoMapValue(VELOCITY_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2021");

      if(maxConfig){
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(CURRENT_DIRECT_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x201B");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(TORQUE_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2022");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(VOLTAGE_QUADRATURE_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2018");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(VOLTAGE_DIRECT_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2019");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(CURRENT_A_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x201C");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(CURRENT_B_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x201D");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(TARGET_TORQUE.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x6071");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(TORQUE_OFFSET.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x60B2");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(DIGITAL_OUTPUTS_SET_VALUE.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2602");
         verifyWorkingCounter(writeSDO(0x1600, 6, computePdoMapValue(ANALOG_OUTPUT_1_VALUE.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x208D");
         verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 15), "failed to write to 0x1600 - 0x5");
      }
      else{
         verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 5), "failed to write to 0x1600 - 0x5");
      }
   }

   public class TPDO_1 extends TxPDO
   {
      public TPDO_1()
      {
         super(0x1A00);
      }

      Unsigned16 statusWord = new Unsigned16(); //0x2011 0, 2 bytes
      Signed8 currentOperationMode = new Signed8(); //0x6061 0, 1 byte
      Signed32 lastError = new Signed32(); //0x200F or 0x580F, 4 bytes
   }

   private void configureTPDO_1()
   {
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 0), "failed to write to 0x1A00 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(CIA_STATUS_WORD.getAddress(), 0, 16)), "failed to write to 0x1A00 - 0x2011");
      verifyWorkingCounter(writeSDO(0x1A00, 2, computePdoMapValue(MODES_OF_OPERATION_DISPLAY.getAddress(), 0, 8)), "failed to write to 0x1A00 - 0x6061");
      verifyWorkingCounter(writeSDO(0x1A00, 3, computePdoMapValue(LAST_ERROR_580F.getAddress(), 0, 32)), "failed to write to 0x1A00 - 0x580F");
//      verifyWorkingCounter(writeSDO(0x1A00, 4, computePdoMapValue(CONTROL_WORD.getAddress(), 0, 16)), "failed to write to 0x1A00 - 0x6040");
//      verifyWorkingCounter(writeSDO(0x1A00, 7, computePdoMapValue(0x2063, 0, 32)), "failed to write to 0x1A00 - 0x2063");


      if(maxConfig){
         verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(MOTOR_TEMPERATURE_VALUE.getAddress(), 0, 32)), "failed to write to 0x1A00 - 0x2063");
         verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(POWER_STAGE_TEMPERATURE_1_VALUE.getAddress(), 0, 32)), "failed to write to 0x1A00 - 0x2061");
         verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(FOLLOWING_ERROR.getAddress(), 0, 32)), "failed to write to 0x1A00 - 0x21EE");
         verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 6), "failed to write to 0x1A00 - 0x8");
      }
      else{
         verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 3), "failed to write to 0x1A00 - 0x8");
      }
   }

   public class TPDO_2 extends TxPDO
   {
      public TPDO_2()
      {
         super(0x1A00);
      }

      Signed32 motorPosition = new Signed32(); //0x6064 0, 4 bytes Tied to position actual, which is the motor position
      Float32 motorVelocity = new Float32(); //0x606C 0, 4 bytes Tied to velocity actual, which is the motor velocity
      Unsigned32 actuatorPosition = new Unsigned32(); //0x2033 0, 4 bytes
//      Float32 actuatorVelocity = new Float32(); //0x2034 0, 4 bytes

      Float32 quadratureCurrent = new Float32(); //0x203B 0, 4 bytes
      Float32 commandedCurrent = new Float32();
      Float32 busVoltage = new Float32(); //0x2060 0, 4 bytes
      //      Float32 motorTemperature = new Float32(); //0x2063, 4 bytes
      Float32 actualTorque = new Float32();
      Float32 temperature = new Float32();
   }

   private void configureTPDO_2()
   {
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 0), "failed to write to 0x1A01 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1A01, 1, computePdoMapValue(0x6064, 0, 32)), "failed to write to 0x1A00 - 0x6064");
      verifyWorkingCounter(writeSDO(0x1A01, 2, computePdoMapValue(0x2031, 0, 32)), "failed to write to 0x1A00 - 0x2031");
      verifyWorkingCounter(writeSDO(0x1A01, 3, computePdoMapValue(0x2033, 0, 32)), "failed to write to 0x1A00 - 0x2033");
//      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(0x2034, 0, 32)), "failed to write to 0x1A00 - 0x2034");
      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(0x203B, 0, 32)), "failed to write to 0x1A00 - 0x203B");
      verifyWorkingCounter(writeSDO(0x1A01, 5, computePdoMapValue(0x2072, 0, 32)), "failed to write to 0x1A00 - 0x2072");
      verifyWorkingCounter(writeSDO(0x1A01, 6, computePdoMapValue(0x2060, 0, 32)), "failed to write to 0x1A00 - 0x2060");
      verifyWorkingCounter(writeSDO(0x1A01, 7, computePdoMapValue(ACTUAL_TORQUE.getAddress(), 0, 32)), "failed to write to 0x1A01 - 0x2029");
      verifyWorkingCounter(writeSDO(0x1A01, 8, computePdoMapValue(POWER_STAGE_TEMPERATURE_1_VALUE.getAddress(), 0, 32)), "failed to write to 0x1A01 - 0x2061");

      if(maxConfig){
         verifyWorkingCounter(writeSDO(0x1A01, 10, computePdoMapValue(0x2034, 0, 32)), "failed to write to 0x1A00 - 0x2034");
         verifyWorkingCounter(writeSDO(0x1A01, 11, computePdoMapValue(0x203C, 0, 32)), "failed to write to 0x1A00 - 0x203C");
         verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 10), "failed to write to 0x1A01 - 0x3");
      }
      else{
         verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 8), "failed to write to 0x1A01 - 0x3");
      }
   }

   public EverestMotorController(int aliasAddress, int position)
   {
      super(VENDOR_ID, PRODUCT_CODE, aliasAddress, position);

      SyncManager syncManager2 = new SyncManager(2, false);
      syncManager2.registerPDO(rpdo);

      SyncManager syncManager3 = new SyncManager(3, false);
      syncManager3.registerPDO(tpdo_1);
      syncManager3.registerPDO(tpdo_2);

      registerSyncManager(syncManager2);
      registerSyncManager(syncManager3);
   }

   public EverestMotorController(int aliasAddress, int position, boolean maxConfig)
   {
      super(VENDOR_ID, PRODUCT_CODE, aliasAddress, position);
      this.maxConfig = maxConfig;
      SyncManager syncManager2 = new SyncManager(2, false);
      syncManager2.registerPDO(rpdo);

      SyncManager syncManager3 = new SyncManager(3, false);
      syncManager3.registerPDO(tpdo_1);
      syncManager3.registerPDO(tpdo_2);

      registerSyncManager(syncManager2);
      registerSyncManager(syncManager3);
   }

   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      configurePDOs();
      readTorqueConstant();
      super.configure(dcEnabled, cycleTimeInNs);
   }

   private void configurePDOs()
   {
      // Configure RPDOs
      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 0), "failed to write to 0x1C12");

      configureRPDO();

      verifyWorkingCounter(writeSDO(0x1C12, 1, (short) 0x1600), "failed to write to 0x1C12 -- 0x1600");

      verifyWorkingCounter(writeSDO(0x1C12, 0, (byte) 1), "failed to write to 0x1C12  -- 0x1");

      // Configure TPDOS
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 0), "failed to write to 0x1C13");

      configureTPDO_1();
      configureTPDO_2();

      verifyWorkingCounter(writeSDO(0x1C13, 1, (short) 0x1A00), "failed to write to 0x1C13 -- 0x1A00");
      verifyWorkingCounter(writeSDO(0x1C13, 2, (short) 0x1A01), "failed to write to 0x1C13 -- 0x1A01");
      verifyWorkingCounter(writeSDO(0x1C13, 0, (byte) 2), "failed to write to 0x1C13 -- 0x2");
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
//      System.out.println(((index & 0xFFFF) << 16) + ((subindex & 0xFF) << 8) + (bitLength & 0xFF));
      return ((index & 0xFFFF) << 16) + ((subindex & 0xFF) << 8) + (bitLength & 0xFF);
   }

   public void readTorqueConstant(){
      tc = readSDOFloat(TORQUE_CONSTANT.getAddress(), 0);
   }

   public double getTorqueConstant(){
      return tc;
   }

   @Override
   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo_1.statusWord;
   }

   @Override
   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo.controlWord;
   }

   public void setOperationMode(byte operationMode)
   {
      rpdo.desiredOperationMode.set(operationMode);
   }

   public void setDesiredCurrent(double desiredCurrent)
   {
      rpdo.quadratureCurrentSetPoint.set((float) desiredCurrent);
   }

   public double getCommandedCurrent()
   {
      return tpdo_2.commandedCurrent.get();
   }

   public void setDesiredPosition(int desiredPosition)
   {
      rpdo.positionSetPoint.set(desiredPosition);
   }

   public void setDesiredVelocity(double desiredVelocity)
   {
      rpdo.velocitySetPoint.set((float) desiredVelocity);
   }

   public int getMotorPosition()
   {
      return tpdo_2.motorPosition.get();
   }

   public double getMotorVelocity()
   {
      return tpdo_2.motorVelocity.get();
   }

   public long getActuatorPosition()
   {
      return tpdo_2.actuatorPosition.get();
   }

   public double getActuatorVelocity()
   {
      return 0.0; //tpdo_2.actuatorVelocity.get();
   }

   public double getQuadratureCurrent()
   {
      return tpdo_2.quadratureCurrent.get();
   }

   public double getBusVoltage()
   {
      return tpdo_2.busVoltage.get();
   }

   public double getMotorTemperature()
   {
      return 0.0; // tpdo_1.motorTemperature.get();
   }

   public byte getCurrentOperationMode()
   {
      return tpdo_1.currentOperationMode.get();
   }

   public int getErrorCode()
   {
      return tpdo_1.lastError.get();
   }

   public double getActualTorque() {
      return tpdo_2.actualTorque.get(); // Nm
   }

   public double getTemperature(){
      return tpdo_2.temperature.get();
   }

   @Override
   public final boolean supportsCA()
   {
      return false;
   }

   @Override
   protected boolean blockLRW()
   {
      return true;
   }
}
