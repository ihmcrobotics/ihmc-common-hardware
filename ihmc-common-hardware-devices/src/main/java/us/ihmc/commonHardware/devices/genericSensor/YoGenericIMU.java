package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.commonHardware.devices.IMUInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoGenericIMU implements YoSensorInterface
{
   // The observed outlier value is on the y axis only, and it's always a value at 17.45 rad/s, that we've observed.
   private static final double GYRO_OUTLIER_THRESHOLD = 15.0;
   private static final double MAX_TICKS_TO_IGNORE_GYRO = 20;

   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;
   protected final IMUInterface imu;

   protected final YoVector3D angularVelocity;
   protected final YoVector3D linearAcceleration;
   protected final YoVector3D unbiasedLinearAcceleration;
   protected final YoVector3D linearAccelerationBias;
   protected final YoVector3D angularVelocityBias;
   protected final YoVector3D unbiasedAngularVelocity;
   protected final YoBoolean gyroDataIgnoredAsOutlier;
   protected final YoInteger gyroDataIgnoredCounts;
   protected final YoDouble imuTemp;

   /**
    * YoWrapper class for an IMU. Reads sensor measurements and
    * updates corresponding yovariables
    *
    * @param prefix         prefix for the named YoVariables in this class
    * @param imu            the Ethersnacks daughter board the IMU is located on
    * @param parentRegistry the initial parent registry for this object
    */
   public YoGenericIMU(String prefix, IMUInterface imu, YoRegistry parentRegistry)
   {
      this.imu = imu;
      registry = new YoRegistry(prefix + name);

      angularVelocity = new YoVector3D(prefix + "AngularVel", registry);
      linearAcceleration = new YoVector3D(prefix + "LinearAccel", registry);
      unbiasedLinearAcceleration = new YoVector3D(prefix + "UnbiasedLinearAccel", registry);
      linearAccelerationBias = new YoVector3D(prefix + "LinearAccelBias", registry);
      unbiasedAngularVelocity = new YoVector3D(prefix + "UnbiasedAngularVel", registry);
      angularVelocityBias = new YoVector3D(prefix + "AngularVelBias", registry);
      gyroDataIgnoredAsOutlier = new YoBoolean(prefix + "GyroDataIgnoredAsOutlier", registry);
      gyroDataIgnoredCounts = new YoInteger(prefix + "GyroDataIgnoredCounts", registry);

      imuTemp = new YoDouble(prefix + "IMUTemp", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      boolean xIsPossiblyOutlier = Math.abs(imu.getGyroX()) > GYRO_OUTLIER_THRESHOLD;
      boolean yIsPossiblyOutlier = Math.abs(imu.getGyroY()) > GYRO_OUTLIER_THRESHOLD;
      boolean zIsPossiblyOutlier = Math.abs(imu.getGyroZ()) > GYRO_OUTLIER_THRESHOLD;
      gyroDataIgnoredAsOutlier.set(checkIfOnlyOneAxisIsOutlier(xIsPossiblyOutlier, yIsPossiblyOutlier, zIsPossiblyOutlier));
      if (gyroDataIgnoredAsOutlier.getBooleanValue())
         gyroDataIgnoredCounts.increment();
      else
         gyroDataIgnoredCounts.set(0);

      // Check if the data is showing up as outliers. If it doesn't show up as an outlier, then update the gyroscope data. Also, if we've been ignoring it for
      // too long, that means it's probably not actually an outlier, so we can update the gyroscope data.
      if (!gyroDataIgnoredAsOutlier.getBooleanValue() || gyroDataIgnoredCounts.getIntegerValue() > MAX_TICKS_TO_IGNORE_GYRO)
         angularVelocity.set(imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());

      linearAcceleration.set(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());

      unbiasedAngularVelocity.add(angularVelocity, angularVelocityBias);
      unbiasedLinearAcceleration.add(linearAcceleration, linearAccelerationBias);

      imuTemp.set(imu.getTemp());
   }

   private static boolean checkIfOnlyOneAxisIsOutlier(boolean xIsPossiblyOutlier, boolean yIsPossiblyOutlier, boolean zIsPossiblyOutlier)
   {
      if (xIsPossiblyOutlier && !yIsPossiblyOutlier && !zIsPossiblyOutlier)
         return true;
      if (yIsPossiblyOutlier && !xIsPossiblyOutlier && !zIsPossiblyOutlier)
         return true;
      if (zIsPossiblyOutlier && !xIsPossiblyOutlier && !yIsPossiblyOutlier)
         return true;
      return false;
   }

   /**
    * Sets linear accelerometer bias. bias is subtracted from the accelerometer measurements to obtain
    * a more accurate measurement
    *
    * @param x bias in the x direction (in sensor frame)
    * @param y bias in the y direction (in sensor frame)
    * @param z bias in the z direction (in sensor frame)
    */
   public void setLinearAccelerationBias(double x, double y, double z)
   {
      linearAccelerationBias.set(x, y, z);
      update();
   }

   public Vector3DReadOnly getLinearAccelerationBias()
   {
      return linearAccelerationBias;
   }

   /**
    * Sets gyroscope bias. bias is subtracted from the gyroscope measurements to obtain
    * a more accurate measurement
    *
    * @param x bias about the x axis (in sensor frame)
    * @param y bias about the y axis (in sensor frame)
    * @param z bias about the z axis (in sensor frame)
    */
   public void setAngularVelocityBias(double x, double y, double z)
   {
      angularVelocityBias.set(x, y, z);
      update();
   }

   public Vector3DReadOnly getAngularVelocityBias()
   {
      return angularVelocityBias;
   }

   /**
    * @return the unbiased angular velocity measurement (bias removed from original signal)
    */
   public Vector3DReadOnly getUnbiasedAngularVelocity()
   {
      return unbiasedAngularVelocity;
   }

   /**
    * @return the angular velocity measurement from the IMU (original signal, but not raw)
    */
   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   /**
    * 
    * @return the unbiased linear acceleration measurement (bias removed from original signal)
    */
   public Vector3DReadOnly getUnbiasedLinearAcceleration()
   {
      return unbiasedLinearAcceleration;
   }

   /**
    *
    * @return the linear acceleration measurement from the IMU (original signal, but not raw)
    */
   public Vector3DReadOnly getLinearAcceleration()
   {
      return linearAcceleration;
   }
}
