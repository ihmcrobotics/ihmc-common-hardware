package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.commonHardware.devices.IMUInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoGenericIMU implements YoSensorInterface
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;
   protected final IMUInterface imu;

   protected final YoFrameVector3D angularVelocity;
   protected final YoFrameVector3D rawAngularVelocity;
   protected final YoFrameVector3D linearAcceleration;
   protected final YoFrameVector3D rawLinearAcceleration;
   protected final YoFrameVector3D unbiasedLinearAcceleration;
   protected final YoFrameVector3D linearAccelerationBias;
   protected final YoFrameVector3D angularVelocityBias;
   protected final YoFrameVector3D unbiasedAngularVelocity;
   protected final YoDouble imuTemp;

   /**
    * YoWrapper class for the IMU component of an Ethersnacks board. Reads sensor measurements and
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

      angularVelocity = new YoFrameVector3D(prefix + "AngularVel", worldFrame, registry);
      rawAngularVelocity = new YoFrameVector3D(prefix + "RawAngularVel", worldFrame, registry);
      linearAcceleration = new YoFrameVector3D(prefix + "LinearAccel", worldFrame, registry);
      rawLinearAcceleration = new YoFrameVector3D(prefix + "RawLinearAccel", worldFrame, registry);
      unbiasedLinearAcceleration = new YoFrameVector3D(prefix + "UnbiasedLinearAccel", worldFrame, registry);
      linearAccelerationBias = new YoFrameVector3D(prefix + "LinearAccelBias", worldFrame, registry);
      unbiasedAngularVelocity = new YoFrameVector3D(prefix + "unbiasedAngularVelocity", worldFrame, registry);
      angularVelocityBias = new YoFrameVector3D(prefix + "AngularVelBias", worldFrame, registry);

      imuTemp = new YoDouble(prefix + "IMUTemp", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {

      angularVelocity.set(imu.getGyroX(), imu.getGyroY(), imu.getGyroZ());
      rawAngularVelocity.set(imu.getRawGyroX(), imu.getRawGyroY(), imu.getRawGyroZ());

      linearAcceleration.set(imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());
      rawLinearAcceleration.set(imu.getRawAccelX(), imu.getRawAccelY(), imu.getRawAccelZ());

      unbiasedAngularVelocity.add(angularVelocity, angularVelocityBias);
      unbiasedLinearAcceleration.add(linearAcceleration, linearAccelerationBias);

      imuTemp.set(imu.getTemp());
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
   }

   public FrameVector3DReadOnly getLinearAccelerationBias()
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
   }

   public FrameVector3DReadOnly getAngularVelocityBias()
   {
      return angularVelocityBias;
   }

   /**
    * 
    * @return the unbiased angular velocity measurement (bias removed from original signal)
    */
   public Vector3DReadOnly getUnbiasedAngularVelocity()
   {
      return unbiasedAngularVelocity;
   }

   /**
    *
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
