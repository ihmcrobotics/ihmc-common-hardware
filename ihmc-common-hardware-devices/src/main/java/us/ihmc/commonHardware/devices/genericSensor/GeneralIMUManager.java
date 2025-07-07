package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
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
      mahonyFilter.setGains(0.1, 5.0e-3);
      mahonyFilter.setYawDriftParameters(0.01, 1.0e-4);
      mahonyYawPitchRoll = new YoFrameYawPitchRoll(prefix + "Mahony", worldFrame, registry);

      Vector3D initialHeading = new Vector3D(Axis3D.X);
      ReferenceFrame.getWorldFrame().transformFromThisToDesiredFrame(imuFrame, initialHeading);
      mahonyFilter.setDesiredInitialHeading(initialHeading);

      filterIMUReadings = new YoBoolean(prefix + "filterH4IMUReadings", registry);

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
      yoIMU.update();

      ImuData measuredIMUData = measuredIMUDataMap.get(name);

      trueNorthInSensorFrame.setIncludingFrame(ReferenceFrame.getWorldFrame(), YoIMUMahonyFilter.NORTH_REFERENCE);
      trueNorthInSensorFrame.changeFrame(imuFrame);

      mahonyFilter.update(yoIMU.getUnbiasedAngularVelocity(), yoIMU.getUnbiasedLinearAcceleration(), trueNorthInSensorFrame);
      mahonyYawPitchRoll.set(mahonyFilter.getEstimatedOrientation());
      quaternion.set(mahonyFilter.getEstimatedOrientation());

      filteredAngularVelocity.update(yoIMU.getUnbiasedAngularVelocity());
      filteredLinearAcceleration.update(yoIMU.getUnbiasedLinearAcceleration());

      YoFrameQuaternion mahoneyFilteredOrientation = mahonyFilter.getEstimatedOrientation();
      YoFrameVector3D mahoneyFilteredAngularVelocity = mahonyFilter.getEstimatedAngularVelocity();
      Vector3DReadOnly mahoneyFilteredLinearAcceleration = yoIMU.getUnbiasedLinearAcceleration();

      if (filterIMUReadings.getBooleanValue())
      {
         measuredIMUData.setLinearAcceleration(filteredLinearAcceleration);
         measuredIMUData.setAngularVelocity(filteredAngularVelocity);
      }
      else
      {
         measuredIMUData.setLinearAcceleration(yoIMU.getUnbiasedLinearAcceleration());
         measuredIMUData.setAngularVelocity(yoIMU.getUnbiasedAngularVelocity());
      }

      if (rootJointEstimate != null)
      {
         computeOrientationAtEstimateFrame(imuFrame, mahoneyFilteredOrientation, rootJointFrame, rootJointEstimate);
         rootJointEstimate.set(rootJointOrientation);
         measuredIMUData.setOrientation(rootJointOrientation);
      }
      else
         measuredIMUData.setOrientation(mahoneyFilteredOrientation);
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
