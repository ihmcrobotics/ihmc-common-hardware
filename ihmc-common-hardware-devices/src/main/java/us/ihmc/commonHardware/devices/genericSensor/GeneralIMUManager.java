package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

/**
 * This class can act as the manager for any generic IMU. It allows for the application of reference frames
 * to transform the raw signals into the correct orientation, as well as compute the quaternion of the imu
 */
public class GeneralIMUManager implements IMUManagerInterface
{
   /**
    * Useful for debugging the main IMU on the pelvis.
    */
   public static boolean COMPUTE_ROOT_JOINT_IMU = false;

   private final IMUDefinition imuDefinition;
   private final YoGenericIMU yoIMU;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame imuFrame;

   private final MovingReferenceFrame rootJointFrame;
   private final YoFrameYawPitchRoll rootJointEstimate;

   private final YoFrameQuaternion quaternion;
   private final YoIMUMahonyFilter mahonyFilter;
   private final YoFrameYawPitchRoll mahonyYawPitchRoll;

   private final FrameVector3D trueNorthInSensorFrame = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final YoBoolean filterIMUReadings;
   private final YoBoolean useMahoneyFilterAngularVelocity;

   private final AlphaFilteredYoFrameVector3D filteredAngularVelocity;
   private final AlphaFilteredYoFrameVector3D filteredLinearAcceleration;

   private final YoRegistry registry;

   private final String name;

   /**
    * Construct the IMU manager
    *
    * @param imuDefinition  Defines the name of the IMU as well as the rigid body connection
    * @param yoIMU          generic IMU yo variable
    * @param dt             controller timestep
    * @param parentRegistry Parent {@code YoRegistry} of the IMU
    */
   public GeneralIMUManager(IMUDefinition imuDefinition, YoGenericIMU yoIMU, double dt, YoRegistry parentRegistry)
   {
      this.imuDefinition = imuDefinition;
      this.yoIMU = yoIMU;

      registry = new YoRegistry(imuDefinition.getName() + getClass().getSimpleName());

      imuFrame = imuDefinition.getIMUFrame();
      name = imuDefinition.getName();

      String prefix = imuDefinition.getName();

      mahonyFilter = new YoIMUMahonyFilter(prefix, prefix + "Mahony", "", dt, true, imuFrame, registry);

      // Initialize the orientation such that the robot faces x+, also apply initial biases
      mahonyFilter.initialize(imuDefinition.getIMUFrame().getTransformToRoot().getRotation(),
                              yoIMU.getAngularVelocityBias().getX(),
                              yoIMU.getAngularVelocityBias().getY(),
                              yoIMU.getAngularVelocityBias().getZ());
      mahonyFilter.setGains(0.5, 0.01);
      mahonyFilter.setYawDriftParameters(0.01, 1.0e-4);
      mahonyYawPitchRoll = new YoFrameYawPitchRoll(prefix + "Mahony", worldFrame, registry);
      quaternion = new YoFrameQuaternion(prefix + "Mahony", worldFrame, registry);

      filterIMUReadings = new YoBoolean(prefix + "filterH4IMUReadings", registry);
      useMahoneyFilterAngularVelocity = new YoBoolean(prefix + "useMahoneyFilterAngularVelocity", registry);
      useMahoneyFilterAngularVelocity.set(true);

      YoDouble h4IMUAngularVelocityFilterAlpha = new YoDouble(prefix + "h4IMUAngularVelocityFilterAlpha", registry);
      YoDouble h4IMULinearAccelerationFilterAlpha = new YoDouble(prefix + "h4IMULinearAccelerationFilterAlpha", registry);

      filteredAngularVelocity = new AlphaFilteredYoFrameVector3D(prefix, "filteredAngularVelocity", registry, h4IMUAngularVelocityFilterAlpha, imuFrame);
      filteredLinearAcceleration = new AlphaFilteredYoFrameVector3D(prefix,
                                                                    "filteredLinearAcceleration",
                                                                    registry,
                                                                    h4IMULinearAccelerationFilterAlpha,
                                                                    imuFrame);

      if (COMPUTE_ROOT_JOINT_IMU)
      {
         rootJointFrame = MultiBodySystemTools.getRootBody(imuDefinition.getRigidBody()).getChildrenJoints().get(0).getFrameAfterJoint();
         rootJointEstimate = new YoFrameYawPitchRoll(imuDefinition.getName() + "RootJointEstimate", rootJointFrame, registry);
      }
      else
      {
         rootJointFrame = null;
         rootJointEstimate = null;
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void read(Map<String, ImuData> measuredIMUDataMap)
   {
      // Update the IMU
      yoIMU.update();

      ImuData measuredIMUDataToPack = measuredIMUDataMap.get(name);

      trueNorthInSensorFrame.setIncludingFrame(ReferenceFrame.getWorldFrame(), YoIMUMahonyFilter.NORTH_REFERENCE);
      trueNorthInSensorFrame.changeFrame(imuFrame);

      // Update the Mahony filter. We pass in regular angular velocity because Mahony class will calculate and account for bias internally
      mahonyFilter.update(yoIMU.getAngularVelocity(), yoIMU.getUnbiasedLinearAcceleration());
      yoIMU.setAngularVelocityBias(mahonyFilter.getIntegralTerm().getX(),
                                   mahonyFilter.getIntegralTerm().getY(),
                                   mahonyFilter.getIntegralTerm().getZ());
      mahonyYawPitchRoll.set(mahonyFilter.getEstimatedOrientation());
      quaternion.set(mahonyFilter.getEstimatedOrientation());

      // Get our estimated orientation, estimated velocity (unbiased), and linear acceleration
      FixedFrameQuaternionBasics orientation = mahonyFilter.getEstimatedOrientation();
      Vector3DReadOnly angularVelocity = useMahoneyFilterAngularVelocity.getBooleanValue() ? mahonyFilter.getEstimatedAngularVelocity() : yoIMU.getUnbiasedAngularVelocity();
      Vector3DReadOnly linearAcceleration = yoIMU.getUnbiasedLinearAcceleration();

      // Filter angular velocity and linear acceleration
      filteredAngularVelocity.update(angularVelocity);
      filteredLinearAcceleration.update(linearAcceleration);

      // Pack the linear acceleration and angular velocity data into measuredIMUDataToPack
      if (filterIMUReadings.getBooleanValue())
      {
         measuredIMUDataToPack.setAngularVelocity(filteredAngularVelocity);
         measuredIMUDataToPack.setLinearAcceleration(filteredLinearAcceleration);
      }
      else
      {
         measuredIMUDataToPack.setAngularVelocity(angularVelocity);
         measuredIMUDataToPack.setLinearAcceleration(linearAcceleration);
      }

      // Pack the orientation data into measuredIMUDataToPack
      measuredIMUDataToPack.setOrientation(orientation);

      // Calculate IMU orientation in parent link frame (visualization purposes only)
      if (rootJointEstimate != null && rootJointFrame != null)
         computeOrientationAtEstimateFrame(imuFrame, orientation, rootJointFrame, rootJointEstimate);
   }

   /**
    * transforms the orientation measurement from the IMU frame to another frame (usually that of the
    * rigid body the IMU is attached to)
    *
    * @param measurementFrame          the sensorFrame of the IMU
    * @param orientationMeasurement    the raw orientation measurement from the IMU
    * @param estimateFrame             the desired frame for the orientation measurement
    * @param orientationEstimateToPack the orientation measurement to pack
    */
   public void computeOrientationAtEstimateFrame(ReferenceFrame measurementFrame,
                                                 Orientation3DReadOnly orientationMeasurement,
                                                 ReferenceFrame estimateFrame,
                                                 Orientation3DBasics orientationEstimateToPack)
   {
      orientationEstimateToPack.setToZero();
      // R_{estimateFrame}^{measurementFrame}
      estimateFrame.transformFromThisToDesiredFrame(measurementFrame, orientationEstimateToPack);

      // R_{estimateFrame}^{world} = R_{measurementFrame}^{world} * R_{estimateFrame}^{measurementFrame}
      orientationEstimateToPack.prepend(orientationMeasurement);
   }

   @Override
   public String getName()
   {
      return name;
   }
}
