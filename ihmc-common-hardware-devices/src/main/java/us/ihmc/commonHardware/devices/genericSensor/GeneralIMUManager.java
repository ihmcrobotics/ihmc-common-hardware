package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   private final QuaternionBasics rootJointOrientation;

   private final FrameVector3D trueNorthInSensorFrame = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final YoBoolean filterIMUReadings;
   private final YoBoolean useMahoneyFilterAngularVelocity;

   private final AlphaFilteredYoFrameVector3D filteredAngularVelocity;
   private final AlphaFilteredYoFrameVector3D filteredLinearAcceleration;

   private final YoRegistry registry;

   private final String name;

   public GeneralIMUManager(IMUDefinition imuDefinition, YoGenericIMU yoIMU, double dt, YoRegistry parentRegistry)
   {
      this.imuDefinition = imuDefinition;
      this.yoIMU = yoIMU;

      registry = new YoRegistry(imuDefinition.getName() + getClass().getSimpleName());

      imuFrame = imuDefinition.getIMUFrame();
      name = imuDefinition.getName();

      String prefix = imuDefinition.getName();

      quaternion = new YoFrameQuaternion(prefix + "quaternion", worldFrame, registry);
      mahonyFilter = new YoIMUMahonyFilter(prefix, prefix + "Mahony", "", dt, true, imuFrame, registry);
      // Initialize the orientation such that the robot faces x+
      mahonyFilter.initialize(imuDefinition.getIMUFrame().getTransformToRoot().getRotation(),
                              yoIMU.getInitialAngularVelocityBias().getX(),
                              yoIMU.getInitialAngularVelocityBias().getY(),
                              yoIMU.getInitialAngularVelocityBias().getZ());
      mahonyFilter.setGains(0.5, 0.01);
      mahonyFilter.setYawDriftParameters(0.01, 1.0e-4);
      mahonyYawPitchRoll = new YoFrameYawPitchRoll(prefix + "Mahony", worldFrame, registry);

//      Vector3D initialHeading = new Vector3D(Axis3D.X);
//      ReferenceFrame.getWorldFrame().transformFromThisToDesiredFrame(imuFrame, initialHeading);
//      mahonyFilter.setDesiredInitialHeading(initialHeading);

      filterIMUReadings = new YoBoolean(prefix + "filterH4IMUReadings", registry);
      useMahoneyFilterAngularVelocity = new YoBoolean(prefix + "useMahoneyFilterAngularVelocity", registry);
      useMahoneyFilterAngularVelocity.set(true);

      YoDouble h4IMUAngularVelocityFilterAlpha = new YoDouble(prefix + "h4IMUAngularVelocityFilterAlpha", registry);
      YoDouble h4IMULinearAccelerationFilterAlpha = new YoDouble(prefix + "h4IMULinearAccelerationFilterAlpha", registry);

      filteredAngularVelocity = new AlphaFilteredYoFrameVector3D(prefix, "filteredAngularVelocity", registry, h4IMUAngularVelocityFilterAlpha, imuFrame);
      filteredLinearAcceleration = new AlphaFilteredYoFrameVector3D(prefix, "filteredLinearAcceleration", registry, h4IMULinearAccelerationFilterAlpha, imuFrame);

      if (COMPUTE_ROOT_JOINT_IMU)
      {
         rootJointFrame = MultiBodySystemTools.getRootBody(imuDefinition.getRigidBody()).getChildrenJoints().get(0).getFrameAfterJoint();
         rootJointEstimate = new YoFrameYawPitchRoll(imuDefinition.getName() + "RootJointEstimate", rootJointFrame, registry);
         rootJointOrientation = new Quaternion();
      }
      else
      {
         rootJointFrame = null;
         rootJointEstimate = null;
         rootJointOrientation = null;
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

//      // Initialize Mahony filter with pre-determined angular velocity biases (if we haven't yet)
//      if (!mahonyFilter.hasBeenInitialized())
//         mahonyFilter.initialize(yoIMU.getUnbiasedLinearAcceleration(),
//                                 trueNorthInSensorFrame,
//                                 yoIMU.getInitialAngularVelocityBias().getX(),
//                                 yoIMU.getInitialAngularVelocityBias().getY(),
//                                 yoIMU.getInitialAngularVelocityBias().getZ());

      // Update the Mahony filter. We pass in regular angular velocity because Mahony class will calculate and account for bias internally
      mahonyFilter.update(yoIMU.getAngularVelocity(), yoIMU.getUnbiasedLinearAcceleration());
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
//      if (rootJointEstimate != null)
//      {
//         computeOrientationAtEstimateFrame(imuFrame, orientation, rootJointFrame, rootJointEstimate);
//         rootJointEstimate.set(rootJointOrientation);
//         measuredIMUDataToPack.setOrientation(rootJointOrientation);
//      }
//      else
//         measuredIMUDataToPack.setOrientation(orientation);
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
      //estimateFrame.transformFromThisToDesiredFrame(measurementFrame, orientationEstimateToPack);
      orientationEstimateToPack.set(imuDefinition.getTransformFromIMUToJoint().getRotation());
      orientationEstimateToPack.invert();

      // R_{estimateFrame}^{world} = R_{measurementFrame}^{world} * R_{estimateFrame}^{measurementFrame}
      orientationEstimateToPack.prepend(orientationMeasurement);
   }

   @Override
   public String getName()
   {
      return name;
   }
}
