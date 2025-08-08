package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.yoVariables.euclid.filters.AlphaFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;
import java.util.Optional;

public class GeneralIMUManager implements IMUManagerInterface
{
   /**
    * Useful for debugging the main IMU on the pelvis.
    */
   public static boolean COMPUTE_ROOT_JOINT_IMU = false;

   /**
    * Useful for finding the offset between the expected and true IMU mounting orientation.
    * This should only be done when robot is on zeroing stand, and it is primarily for the
    * pelvis IMU which, when zeroing the robot, usually needs to be perfectly level in world
    * with 0.0 x and y linear acceleration and a linear acceleration in z equivalent to gravity
    */
   public static boolean DEBUG_IMU_ORIENTATION_OFFSETS = false;

   private final YoGenericIMU yoIMU;
   private final ReferenceFrame imuFrame;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame originalIMUFrame;
   private final Optional<YoFramePoseUsingYawPitchRoll> imuCorrectionOffset;
   private final Optional<ReferenceFrame> correctedIMUFrame;

   private final MovingReferenceFrame rootJointFrame;
   private final YoFrameYawPitchRoll rootJointEstimate;

   private final YoFrameQuaternion orientationInIMUFrame;
   private final YoFrameVector3D angularVelocityInIMUFrame;
   private final YoFrameVector3D linearAccelerationInIMUFrame;

   private final Optional<YoFrameQuaternion> orientationInCorrectedIMUFrame;
   private final Optional<YoFrameVector3D> angularVelocityInCorrectedIMUFrame;
   private final Optional<YoFrameVector3D> linearAccelerationInCorrectedIMUFrame;

   private final YoIMUMahonyFilter mahonyFilter;
   private final YoFrameYawPitchRoll mahonyYawPitchRoll;

   private final YoBoolean filterIMUReadings;
   private final YoBoolean useMahoneyFilterAngularVelocity;

   private final YoFrameVector3D linearAccelerationInWorld;
   private final YoDouble accelerationMagnitude;

   private final AlphaFilteredYoFrameVector3D filteredAngularVelocity;
   private final AlphaFilteredYoFrameVector3D filteredLinearAcceleration;

   private final SimpleMovingAverageFilteredYoVariable[] averagedIMUs;

   private final YoBoolean collectAverages;
   private final YoBoolean isAveraging;

   private final YoRegistry registry;

   private final String name;

   public GeneralIMUManager(IMUDefinition imuDefinition, YoGenericIMU yoIMU, double dt, YoRegistry parentRegistry)
   {
      this.yoIMU = yoIMU;

      registry = new YoRegistry(imuDefinition.getName() + getClass().getSimpleName());

      originalIMUFrame = imuDefinition.getIMUFrame();
      name = imuDefinition.getName();
      String prefix = name;

      orientationInIMUFrame = new YoFrameQuaternion(prefix + "OrientationInIMUFrame", originalIMUFrame, registry);
      angularVelocityInIMUFrame = new YoFrameVector3D(prefix + "AngularVelocityInIMUFrame", originalIMUFrame, registry);
      linearAccelerationInIMUFrame = new YoFrameVector3D(prefix + "LinearAccelerationInIMUFrame", originalIMUFrame, registry);

      // These are for finding the offset between the expected and true IMU mounting orientation
      if (DEBUG_IMU_ORIENTATION_OFFSETS)
      {
         imuCorrectionOffset = Optional.of(new YoFramePoseUsingYawPitchRoll(prefix + "IMUCorrectionOffset", originalIMUFrame.getParent(), registry));
         imuCorrectionOffset.get().set(originalIMUFrame.getTransformToParent());
         correctedIMUFrame = Optional.of(new ReferenceFrame(prefix + "CorrectedIMUFrame", originalIMUFrame.getParent())
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.set(imuCorrectionOffset.get());
            }
         });

         orientationInCorrectedIMUFrame = Optional.of(new YoFrameQuaternion(prefix + "OrientationInCorrectedIMUFrame", correctedIMUFrame.get(), registry));
         angularVelocityInCorrectedIMUFrame = Optional.of(new YoFrameVector3D(prefix + "AngularVelocityInCorrectedIMUFrame", correctedIMUFrame.get(), registry));
         linearAccelerationInCorrectedIMUFrame = Optional.of(new YoFrameVector3D(prefix + "LinearAccelerationInCorrectedIMUFrame", correctedIMUFrame.get(), registry));

         imuFrame = correctedIMUFrame.get();
      }
      else
      {
         imuCorrectionOffset = Optional.empty();
         correctedIMUFrame = Optional.empty();
         orientationInCorrectedIMUFrame = Optional.empty();
         angularVelocityInCorrectedIMUFrame = Optional.empty();
         linearAccelerationInCorrectedIMUFrame = Optional.empty();
         imuFrame = originalIMUFrame;
      }

      // Create Mahony filter for orientation estimation
      mahonyFilter = new YoIMUMahonyFilter(prefix, prefix + "Mahony", "", dt, true, imuFrame, registry);

      // Initialize mahony orientation such that the robot faces x+, also apply initial biases
      mahonyFilter.initialize(imuFrame.getTransformToRoot().getRotation(),
                              yoIMU.getAngularVelocityBias().getX(),
                              yoIMU.getAngularVelocityBias().getY(),
                              yoIMU.getAngularVelocityBias().getZ());
      mahonyFilter.setGains(0.5, 0.01);
      mahonyFilter.setYawDriftParameters(0.01, 1.0e-4);
      mahonyYawPitchRoll = new YoFrameYawPitchRoll(prefix + "Mahony", worldFrame, registry);

      useMahoneyFilterAngularVelocity = new YoBoolean(prefix + "useMahonyFilterAngularVelocity", registry);
      useMahoneyFilterAngularVelocity.set(true);

      correctedIMUFrame.ifPresent(value -> value.addListener((v) -> mahonyFilter.initialize(imuFrame.getTransformToRoot().getRotation(),
                                                                                            yoIMU.getAngularVelocityBias().getX(),
                                                                                            yoIMU.getAngularVelocityBias().getY(),
                                                                                            yoIMU.getAngularVelocityBias().getZ())));

      // This is for recording a filtered steady state signal of linear accel and angular vel to estimate biases
      averagedIMUs = new SimpleMovingAverageFilteredYoVariable[6];
      averagedIMUs[0] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qddX", 50, registry);
      averagedIMUs[1] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qddY", 50, registry);
      averagedIMUs[2] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qddZ", 50, registry);
      averagedIMUs[3] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qdwX", 50, registry);
      averagedIMUs[4] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qdwY", 50, registry);
      averagedIMUs[5] = new SimpleMovingAverageFilteredYoVariable(prefix + "Avg_qdwZ", 50, registry);

      collectAverages = new YoBoolean(prefix + "CollectAveragesForBiasEstimation", registry);
      collectAverages.addListener(v ->
                                  {
                                     beginAveraging();
                                     collectAverages.set(false, false);
                                  });
      isAveraging = new YoBoolean(prefix + "IsAveraging", registry);

      // YoVariables for filtering the IMU signals
      filterIMUReadings = new YoBoolean(prefix + "filterH4IMUReadings", registry);

      YoDouble h4IMUAngularVelocityFilterAlpha = new YoDouble(prefix + "h4IMUAngularVelocityFilterAlpha", registry);
      YoDouble h4IMULinearAccelerationFilterAlpha = new YoDouble(prefix + "h4IMULinearAccelerationFilterAlpha", registry);

      filteredAngularVelocity = new AlphaFilteredYoFrameVector3D(prefix, "filteredAngularVelocity", registry, h4IMUAngularVelocityFilterAlpha,
                                                                 imuFrame);
      filteredLinearAcceleration = new AlphaFilteredYoFrameVector3D(prefix, "filteredLinearAcceleration", registry, h4IMULinearAccelerationFilterAlpha,
                                                                    imuFrame);

      linearAccelerationInWorld = new YoFrameVector3D(prefix + "LinearAccelerationInWorld", worldFrame, registry);
      accelerationMagnitude = new YoDouble(prefix + "AccelerationMagnitude", registry);

      // For debugging
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

      correctedIMUFrame.ifPresent(ReferenceFrame::update);

      ImuData measuredIMUDataToPack = measuredIMUDataMap.get(name);

      // Update the Mahony filter. We pass in regular angular velocity because Mahony class will calculate and account for bias internally
      mahonyFilter.update(yoIMU.getAngularVelocity(), yoIMU.getUnbiasedLinearAcceleration());
      yoIMU.setAngularVelocityBias(mahonyFilter.getIntegralTerm().getX(),
                                   mahonyFilter.getIntegralTerm().getY(),
                                   mahonyFilter.getIntegralTerm().getZ());
      mahonyYawPitchRoll.set(mahonyFilter.getEstimatedOrientation());

      // Get our estimated orientation, estimated velocity (unbiased), and linear acceleration (unbiased)
      Tuple4DReadOnly orientation = mahonyFilter.getEstimatedOrientation();
      Vector3DReadOnly angularVelocity = useMahoneyFilterAngularVelocity.getBooleanValue() ? mahonyFilter.getEstimatedAngularVelocity() : yoIMU.getUnbiasedAngularVelocity();
      Vector3DReadOnly linearAcceleration = yoIMU.getUnbiasedLinearAcceleration();

      // Set the imu signals in the original, expected IMU frames provided by the URDF
      orientationInIMUFrame.setMatchingFrame(imuFrame, orientation);
      angularVelocityInIMUFrame.setMatchingFrame(imuFrame, angularVelocity);
      linearAccelerationInIMUFrame.setMatchingFrame(imuFrame, linearAcceleration);

      // Set the imu signals in the corrected IMU frames that we hand tuned
      orientationInCorrectedIMUFrame.ifPresent(value -> value.setMatchingFrame(imuFrame, orientation));
      angularVelocityInCorrectedIMUFrame.ifPresent(value -> value.setMatchingFrame(imuFrame, angularVelocity));
      linearAccelerationInCorrectedIMUFrame.ifPresent(value -> value.setMatchingFrame(imuFrame, linearAcceleration));

      // These are useful for debugging and seeing if the measured gravity vector is in the right place
      linearAccelerationInWorld.setMatchingFrame(imuFrame, yoIMU.getUnbiasedLinearAcceleration());
      accelerationMagnitude.set(linearAccelerationInWorld.norm());

      // Filter angular velocity and linear acceleration
      // Here we use signal in original IMU frame since high-level control will assume that from URDF
      filteredAngularVelocity.update(angularVelocityInIMUFrame);
      filteredLinearAcceleration.update(linearAccelerationInIMUFrame);

      // Pack the orientation data into measuredIMUDataToPack
      // Here we use signal in original IMU frame since high-level control will assume that from URDF
      measuredIMUDataToPack.setOrientation(orientationInIMUFrame);

      // Pack the linear acceleration and angular velocity data into measuredIMUDataToPack
      // Here we use signal in original IMU frame since high-level control will assume that from URDF
      if (filterIMUReadings.getBooleanValue())
      {
         measuredIMUDataToPack.setAngularVelocity(filteredAngularVelocity);
         measuredIMUDataToPack.setLinearAcceleration(filteredLinearAcceleration);
      }
      else
      {
         measuredIMUDataToPack.setAngularVelocity(angularVelocityInIMUFrame);
         measuredIMUDataToPack.setLinearAcceleration(linearAccelerationInIMUFrame);
      }

      // Collect steady-state averages from IMU signals to estimate biases by hand
      if (isAveraging.getBooleanValue())
      {
         boolean finishedAveraging = updateAveraging();
         if (finishedAveraging)
         {
            yoIMU.setAngularVelocityBias(-averagedIMUs[3].getDoubleValue(), -averagedIMUs[4].getDoubleValue(), -averagedIMUs[5].getDoubleValue());
            mahonyFilter.initialize(imuFrame.getTransformToRoot().getRotation(),
                                    yoIMU.getAngularVelocityBias().getX(),
                                    yoIMU.getAngularVelocityBias().getY(),
                                    yoIMU.getAngularVelocityBias().getZ());
            isAveraging.set(false);
         }
      }

      // Calculate IMU orientation in parent link frame (visualization purposes only)
      if (rootJointEstimate != null && rootJointFrame != null)
         computeOrientationAtEstimateFrame(imuFrame, orientationInIMUFrame, rootJointFrame, rootJointEstimate);
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

   private void beginAveraging()
   {
      for (SimpleMovingAverageFilteredYoVariable averagedIMU : averagedIMUs)
         averagedIMU.reset();
      isAveraging.set(true);
   }

   private boolean updateAveraging()
   {
      averagedIMUs[0].update(yoIMU.getLinearAcceleration().getX());
      averagedIMUs[1].update(yoIMU.getLinearAcceleration().getY());
      averagedIMUs[2].update(yoIMU.getLinearAcceleration().getZ());
      averagedIMUs[3].update(yoIMU.getAngularVelocity().getX());
      averagedIMUs[4].update(yoIMU.getAngularVelocity().getY());
      averagedIMUs[5].update(yoIMU.getAngularVelocity().getZ());

      return averagedIMUs[0].getHasBufferWindowFilled();
   }

   @Override
   public String getName()
   {
      return name;
   }
}
