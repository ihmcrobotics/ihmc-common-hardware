package us.ihmc.commonHardware.devices.etherCATDevices.h4;

import us.ihmc.commonHardware.devices.IMUInterface;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.EtherCATDeviceStatusProvider;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;

public class H4IMU extends Slave implements IMUInterface, EtherCATDeviceStatusProvider
{
   private static final int VENDOR_ID = 0x1011;
   private static final int PRODUCT_CODE = 0x00000200;

   public static double RAW_ACCEL_TO_G = 0.000244;
   public static double RAW_GYRO_TO_RAD_PER_SEC = (2000.0 * (Math.PI * 2.0) / 360.0) / 65535.0;
   public static double RAW_TEMP_TO_CELCIUS_SCALAR = 1.0 / 132.48;
   public static double RAW_TEMP_TO_CELCIUS_CONSTANT = 25.0;
   public static double GRAVITY = 9.80665;

   private final IMUData imuData = new IMUData();
   private final IMUControl imuControl = new IMUControl();

   class IMUData extends TxPDO
   {
      Signed16 accelX = new Signed16();
      Signed16 accelY = new Signed16();
      Signed16 accelZ = new Signed16();
      Signed16 gyroX = new Signed16();
      Signed16 gyroY = new Signed16();
      Signed16 gyroZ = new Signed16();
      Signed16 imuTemp = new Signed16();
      Signed16 boardTemp = new Signed16();
      Unsigned16 SlaveCycleCounter = new Unsigned16();
      Unsigned16 slaveCycleTime = new Unsigned16();

      protected IMUData()
      {
         super(0x1a00);
      }
   }

   class IMUControl extends RxPDO
   {
      Unsigned8 reset = new Unsigned8();

      protected IMUControl()
      {
         super(0x1600);
      }
   }

   public H4IMU(int aliasAddress, int position)
   {
      super(VENDOR_ID, PRODUCT_CODE, aliasAddress, position);

      registerSyncManager(new SyncManager(2, false));
      registerSyncManager(new SyncManager(3, false));

      sm(2).registerPDO(imuControl);
      sm(3).registerPDO(imuData);
   }

   @Override
   public double getRawTemp()
   {
      return imuData.imuTemp.get();
   }

   @Override
   public double getTemp()
   {
      return imuData.imuTemp.get() * RAW_TEMP_TO_CELCIUS_SCALAR + RAW_TEMP_TO_CELCIUS_CONSTANT;
   }

   public double getRawBoardTemp()
   {
      return imuData.boardTemp.get();
   }

   public double getBoardTemp()
   {
      return imuData.boardTemp.get() * RAW_TEMP_TO_CELCIUS_SCALAR + RAW_TEMP_TO_CELCIUS_CONSTANT;
   }

   @Override
   public double getRawAccelX()
   {
      return imuData.accelX.get();
   }

   @Override
   public double getRawAccelY()
   {
      return imuData.accelY.get();
   }

   @Override
   public double getRawAccelZ()
   {
      return imuData.accelZ.get();
   }

   @Override
   public double getAccelX()
   {
      return (getRawAccelX() * RAW_ACCEL_TO_G) * GRAVITY;
   }

   @Override
   public double getAccelY()
   {
      return (getRawAccelY() * RAW_ACCEL_TO_G) * GRAVITY;
   }

   @Override
   public double getAccelZ()
   {
      return (getRawAccelZ() * RAW_ACCEL_TO_G) * GRAVITY;
   }

   @Override
   public double getRawGyroX()
   {
      return imuData.gyroX.get();
   }

   @Override
   public double getRawGyroY()
   {
      return imuData.gyroY.get();
   }

   @Override
   public double getRawGyroZ()
   {
      return imuData.gyroZ.get();
   }

   @Override
   public double getGyroX()
   {
      return getRawGyroX() * RAW_GYRO_TO_RAD_PER_SEC;
   }

   @Override
   public double getGyroY()
   {
      return getRawGyroY() * RAW_GYRO_TO_RAD_PER_SEC;
   }

   @Override
   public double getGyroZ()
   {
      return getRawGyroZ() * RAW_GYRO_TO_RAD_PER_SEC;
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

   public int getCycleCount()
   {
      return imuData.SlaveCycleCounter.get();
   }

   public int getCycleTime()
   {
      return imuData.slaveCycleTime.get();
   }
}
