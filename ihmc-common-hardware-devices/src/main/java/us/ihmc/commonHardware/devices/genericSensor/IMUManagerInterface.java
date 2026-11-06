package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.sensorProcessing.outputData.ImuData;

import java.util.Map;

public interface IMUManagerInterface
{
   /**
    * Reads the data from the IMU and places into the corresponding {@code ImuData}
    *
    * @param measuredIMUData Map with imu names and data
    */
   void read(Map<String, ImuData> measuredIMUData);

   String getName();

   /**
    * Sets the gyroscope bias. This value is added to the raw angular velocity measurement to obtain
    * the unbiased angular velocity, i.e. it is the correction, not the sensor's raw offset.
    *
    * @param bias the gyroscope bias to apply, in the IMU's own sensor frame
    */
   void setGyroscopeBias(Vector3DReadOnly bias);

   /**
    * Sets the accelerometer bias. This value is added to the raw linear acceleration measurement to
    * obtain the unbiased linear acceleration, i.e. it is the correction, not the sensor's raw offset.
    *
    * @param bias the accelerometer bias to apply, in the IMU's own sensor frame
    */
   void setAccelerometerBias(Vector3DReadOnly bias);

   /**
    * @return the gyroscope bias currently being applied to this IMU's angular velocity measurement
    */
   Vector3DReadOnly getGyroscopeBias();

   /**
    * @return the accelerometer bias currently being applied to this IMU's linear acceleration
    *         measurement
    */
   Vector3DReadOnly getAccelerometerBias();
}
