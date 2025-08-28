package us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks;

import us.ihmc.commonHardware.devices.IMUInterface;
import us.ihmc.etherCAT.master.RxPDO;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.SyncManager;
import us.ihmc.etherCAT.master.TxPDO;
import hardwareStatusUI.controllerSide.EtherCATDeviceStatusProvider;

public class EtherSnacksTDKIMU extends Slave implements IMUInterface, EtherCATDeviceStatusProvider
{
   private static final int VENDOR_ID = 0x00000603;
   private static final int PRODUCT_CODE = 0x10003103;

   //The sensitivity of the accelerometer is set to 8g:
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
      Unsigned16 temp = new Unsigned16();
      Unsigned16 status = new Unsigned16();

      protected IMUData()
      {
         super(0x1a00);
      }
   }

   class IMUControl extends RxPDO
   {
      Unsigned16 config = new Unsigned16();

      protected IMUControl()
      {
         super(0x1600);
      }
   }

   public EtherSnacksTDKIMU(int aliasAddress, int position)
   {
      super(VENDOR_ID, PRODUCT_CODE, aliasAddress, position);

      registerSyncManager(new SyncManager(2, false));
      registerSyncManager(new SyncManager(3, false));

      sm(2).registerPDO(imuControl);
      sm(3).registerPDO(imuData);
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
      return imuData.gyroX.get() * RAW_GYRO_TO_RAD_PER_SEC;
   }

   @Override
   public double getGyroY()
   {
      return imuData.gyroY.get() * RAW_GYRO_TO_RAD_PER_SEC;
   }

   @Override
   public double getGyroZ()
   {
      return imuData.gyroZ.get() * RAW_GYRO_TO_RAD_PER_SEC;
   }

   @Override
   public double getRawTemp()
   {
      return imuData.temp.get();
   }

   @Override
   public double getTemp()
   {
      return imuData.temp.get() * RAW_TEMP_TO_CELCIUS_SCALAR + RAW_TEMP_TO_CELCIUS_CONSTANT;
   }

   public int getRawStatus()
   {
      return imuData.status.get();
   }

   @Override
   public boolean isResponding()
   {
      return super.isOperational();
   }
}
