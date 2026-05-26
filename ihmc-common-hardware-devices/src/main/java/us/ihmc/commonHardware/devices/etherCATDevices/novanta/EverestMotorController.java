package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

import us.ihmc.etherCAT.javalution.Struct.Unsigned16;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import us.ihmc.etherCAT.slaves.DSP402Slave;

import static us.ihmc.commonHardware.devices.etherCATDevices.novanta.EverestRegisters.*;

public class EverestMotorController extends DSP402Slave
{
   private RPDO rpdo = new RPDO();
   private TPDO_1 tpdo_1 = new TPDO_1();
   private TPDO_2 tpdo_2 = new TPDO_2();

   public class RPDO extends RxPDO
   {
      public RPDO()
      {
         super(0x1600);
      }

      Unsigned16 desiredControlWord = new Unsigned16(); //0x2010 or 0x6040
      Unsigned16 desiredOperationMode = new Unsigned16(); //0x6060 (0x2014 holds a more complex version)
      Float32 quadratureCurrentSetPoint = new Float32(); //0x201A
      Signed32 positionSetPoint = new Signed32();
      Float32 velocitySetPoint = new Float32();


   }

   private void configureRPDO()
   {
      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1600, 1, computePdoMapValue(CONTROL_WORD.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1600, 2, computePdoMapValue(MODES_OF_OPERATION.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1600, 3, computePdoMapValue(CURRENT_QUADRATURE_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1600, 4, computePdoMapValue(POSITION_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1600, 5, computePdoMapValue(VELOCITY_SET_POINT.getAddress(), 0, 32)), "failed to write to 0x1600 - 0x2010");


      verifyWorkingCounter(writeSDO(0x1600, 0, (byte) 5), "failed to write to 0x1600 - 0x3");
   }

   public class TPDO_1 extends TxPDO
   {
      public TPDO_1()
      {
         super(0x1A00);
      }

      Signed32 motorPosition = new Signed32(); //0x2030 0, 4 bytes
      Float32 motorVelocity = new Float32(); //0x2031 0, 4 bytes
      Unsigned32 actuatorPosition = new Unsigned32(); //0x2033 0, 4 bytes
      Float32 actuatorVelocity = new Float32(); //0x2034 0, 4 bytes

      Float32 quadratureCurrent = new Float32(); //0x203B 0, 4 bytes
      Float32 busVoltage = new Float32(); //0x2060 0, 4 bytes
      Float32 motorTemperature = new Float32(); //0x2063, 4 bytes
   }

   private void configureTPDO_1()
   {
      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1A00, 1, computePdoMapValue(0x2030, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 2, computePdoMapValue(0x2031, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 3, computePdoMapValue(0x2033, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 4, computePdoMapValue(0x2034, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 5, computePdoMapValue(0x203B, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 6, computePdoMapValue(0x2060, 0, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A00, 7, computePdoMapValue(0x2063, 0, 32)), "failed to write to 0x1600 - 0x2010");

      verifyWorkingCounter(writeSDO(0x1A00, 0, (byte) 7), "failed to write to 0x1600 - 0x3");
   }

   public class TPDO_2 extends TxPDO
   {
      public TPDO_2()
      {
         super(0x1A00);
      }


      Unsigned16 statusWord = new Unsigned16(); //0x2011 0, 2 bytes
      Signed8 currentOperationMode = new Signed8(); //0x6061 0, 1 byte
      Signed32 lastError = new Signed32(); //0x200F or 0x580F, 4 bytes
      Unsigned16 currentControlWord = new Unsigned16();
   }

   private void configureTPDO_2()
   {
      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 0), "failed to write to 0x1600 - 0x0"); // disable 0x1600 while we write

      verifyWorkingCounter(writeSDO(0x1A01, 1, computePdoMapValue(STATUS_WORD.getAddress(), 0, 16)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A01, 2, computePdoMapValue(MODES_OF_OPERATION_DISPLAY.getAddress(), 0, 8)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A01, 3, computePdoMapValue(LAST_ERROR.getAddress(), 1, 32)), "failed to write to 0x1600 - 0x2010");
      verifyWorkingCounter(writeSDO(0x1A01, 4, computePdoMapValue(CIA_CONTROL_WORD.getAddress(), 1, 16)), "failed to write to 0x1600 - 0x2010");


      verifyWorkingCounter(writeSDO(0x1A01, 0, (byte) 4), "failed to write to 0x1600 - 0x3");
   }

   public EverestMotorController(int aliasAddress, int position)
   {
      super(0, 0, aliasAddress, position);

      SyncManager syncManager2 = new SyncManager(2, false);
      syncManager2.registerPDO(rpdo);

      SyncManager syncManager3 = new SyncManager(3, false);
      syncManager3.registerPDO(tpdo_1);
      syncManager3.registerPDO(tpdo_2);

      registerSyncManager(syncManager2);
      registerSyncManager(syncManager3);
   }

   @Override
   protected void configure(boolean dcEnabled, long cycleTimeInNs)
   {
      configurePDOs();

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
      return ((index & 0xFFFF) << 16) + ((subindex & 0xFF) << 8) + (bitLength & 0xFF);
   }

   public void setControlWord(int controlWord)
   {
      rpdo.desiredControlWord.set(controlWord);
   }

   @Override
   protected Unsigned16 getStatusWordPDOEntry()
   {
      return tpdo_2.statusWord;
   }

   @Override
   protected Unsigned16 getControlWordPDOEntry()
   {
      return rpdo.desiredControlWord;
   }

   public void setOperationMode(int operationMode)
   {
      rpdo.desiredOperationMode.set(operationMode);
   }

   public void setDesiredCurrent(double desiredCurrent)
   {
      rpdo.quadratureCurrentSetPoint.set((float) desiredCurrent);
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
      return tpdo_1.motorPosition.get();
   }

   public double getMotorVelocity()
   {
      return tpdo_1.motorVelocity.get();
   }

   public long getActuatorPosition()
   {
      return tpdo_1.actuatorPosition.get();
   }

   public double getActuatorVelocity()
   {
      return tpdo_1.actuatorVelocity.get();
   }

   public double getQuadratureCurrent()
   {
      return tpdo_1.quadratureCurrent.get();
   }

   public double getBusVoltage()
   {
      return tpdo_1.busVoltage.get();
   }

   public double getMotorTemperature()
   {
      return tpdo_1.motorTemperature.get();
   }

   public int getCurrentControlWord()
   {
      return tpdo_2.currentControlWord.get();
   }

   public int getCurrentOperationMode()
   {
      return tpdo_2.currentOperationMode.get();
   }

   public int getErrorCode()
   {
      return tpdo_2.lastError.get();
   }
}
