package us.ihmc.commonHardware.devices.genericSensor;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.tools.Timer;
import us.ihmc.yoVariables.euclid.YoVector3D;
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

public class GeneralIMUManager implements IMUManagerInterface
{
   /**
    * Useful for debugging the main IMU on the pelvis.
    */
   public static boolean COMPUTE_ROOT_JOINT_IMU = false;

   private final IMUDefinition imuDefinition;
   private final YoGenericIMU yoIMU;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFramePoseUsingYawPitchRoll imuOffset;
   private final ReferenceFrame imuOffsetFrame;

   private final MovingReferenceFrame rootJointFrame;
   private final YoFrameYawPitchRoll rootJointEstimate;

   private final YoFrameQuaternion quaternion;
   private final YoIMUMahonyFilter mahonyFilter;
   private final YoFrameYawPitchRoll mahonyYawPitchRoll;

   private final YoBoolean filterIMUReadings;
   private final YoBoolean useMahoneyFilterAngularVelocity;

   private final YoFrameVector3D adjustedLinearAccelerationInWorld;
   private final YoDouble accelerationMagnitude;

   private final AlphaFilteredYoFrameVector3D filteredAngularVelocity;
   private final AlphaFilteredYoFrameVector3D filteredLinearAcceleration;

   private final Vector3D idealVector = new Vector3D(0.0, 0.0, 9.81);

   private final SimpleMovingAverageFilteredYoVariable[] averagedIMU;

   private final YoBoolean collectAverages;
   private final YoBoolean isAveraging;

   private final YoRegistry registry;

   private final String name;

   public GeneralIMUManager(IMUDefinition imuDefinition, YoGenericIMU yoIMU, double dt, YoRegistry parentRegistry)
   {
      this.imuDefinition = imuDefinition;
      this.yoIMU = yoIMU;

      registry = new YoRegistry(imuDefinition.getName() + getClass().getSimpleName());

      ReferenceFrame imuFrame = imuDefinition.getIMUFrame();
      name = imuDefinition.getName();
      String prefix = imuDefinition.getName();

      imuOffset = new YoFramePoseUsingYawPitchRoll(prefix + "IMUOffset", imuFrame, registry);
      imuOffsetFrame = new ReferenceFrame(prefix + "IMUOffsetFrame", imuFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(imuOffset);
         }
      };

      mahonyFilter = new YoIMUMahonyFilter(prefix, prefix + "Mahony", "", dt, true, imuOffsetFrame, registry);

      // Initialize the orientation such that the robot faces x+, also apply initial biases
      mahonyFilter.initialize(imuOffsetFrame.getTransformToRoot().getRotation(),
                              yoIMU.getAngularVelocityBias().getX(),
                              yoIMU.getAngularVelocityBias().getY(),
                              yoIMU.getAngularVelocityBias().getZ());
      mahonyFilter.setGains(0.5, 0.01);
      mahonyFilter.setYawDriftParameters(0.01, 1.0e-4);
      mahonyYawPitchRoll = new YoFrameYawPitchRoll(prefix + "Mahony", worldFrame, registry);
      quaternion = new YoFrameQuaternion(prefix + "Mahony", worldFrame, registry);

      imuOffsetFrame.addListener((v) ->
            mahonyFilter.initialize(imuOffsetFrame.getTransformToRoot().getRotation(),
                                    yoIMU.getAngularVelocityBias().getX(),
                                    yoIMU.getAngularVelocityBias().getY(),
                                    yoIMU.getAngularVelocityBias().getZ() ));

      filterIMUReadings = new YoBoolean(prefix + "filterH4IMUReadings", registry);
      useMahoneyFilterAngularVelocity = new YoBoolean(prefix + "useMahoneyFilterAngularVelocity", registry);
      useMahoneyFilterAngularVelocity.set(true);

      averagedIMU = new SimpleMovingAverageFilteredYoVariable[6];
      averagedIMU[0] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qddX", 50, registry);
      averagedIMU[1] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qddY", 50, registry);
      averagedIMU[2] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qddZ", 50, registry);
      averagedIMU[3] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qdwX", 50, registry);
      averagedIMU[4] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qdwY", 50, registry);
      averagedIMU[5] = new SimpleMovingAverageFilteredYoVariable(name + "Avg_qdwZ", 50, registry);

      collectAverages = new YoBoolean(name + "CollectAverages", registry);
      isAveraging = new YoBoolean(name + "IsAveraging", registry);

      YoDouble h4IMUAngularVelocityFilterAlpha = new YoDouble(prefix + "h4IMUAngularVelocityFilterAlpha", registry);
      YoDouble h4IMULinearAccelerationFilterAlpha = new YoDouble(prefix + "h4IMULinearAccelerationFilterAlpha", registry);

      filteredAngularVelocity = new AlphaFilteredYoFrameVector3D(prefix, "filteredAngularVelocity", registry, h4IMUAngularVelocityFilterAlpha, imuOffsetFrame);
      filteredLinearAcceleration = new AlphaFilteredYoFrameVector3D(prefix, "filteredLinearAcceleration", registry, h4IMULinearAccelerationFilterAlpha, imuOffsetFrame);

      adjustedLinearAccelerationInWorld = new YoFrameVector3D(prefix + "AdjustedLinearAccelerationInWorld", ReferenceFrame.getWorldFrame(), registry);
      accelerationMagnitude = new YoDouble(prefix + "AccelerationMagnitude", registry);
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
      imuOffsetFrame.update();

      ImuData measuredIMUDataToPack = measuredIMUDataMap.get(name);

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

      if(collectAverages.getBooleanValue())
      {
         beginAveraging();
         collectAverages.set(false);
      }

      if (isAveraging.getBooleanValue())
      {
         boolean finishedAveraging = updateAveraging();
         if (finishedAveraging)
         {
            yoIMU.angularVelocityBias.set(-averagedIMU[3].getDoubleValue(), -averagedIMU[4].getDoubleValue(), -averagedIMU[5].getDoubleValue());
            isAveraging.set(false);
         }
      }

      adjustedLinearAccelerationInWorld.setMatchingFrame(imuOffsetFrame, yoIMU.linearAcceleration);
      accelerationMagnitude.set(adjustedLinearAccelerationInWorld.norm());

      // Pack the orientation data into measuredIMUDataToPack
      measuredIMUDataToPack.setOrientation(orientation);

      // Calculate IMU orientation in parent link frame (visualization purposes only)
      if (rootJointEstimate != null && rootJointFrame != null)
         computeOrientationAtEstimateFrame(imuOffsetFrame, orientation, rootJointFrame, rootJointEstimate);
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
      for (SimpleMovingAverageFilteredYoVariable averagedForce : averagedIMU)
         averagedForce.reset();
      isAveraging.set(true);
   }

   private boolean updateAveraging()
   {
      averagedIMU[0].update(yoIMU.linearAcceleration.getX());
      averagedIMU[1].update(yoIMU.linearAcceleration.getY());
      averagedIMU[2].update(yoIMU.linearAcceleration.getZ());
      averagedIMU[3].update(yoIMU.angularVelocity.getX());
      averagedIMU[4].update(yoIMU.angularVelocity.getY());
      averagedIMU[5].update(yoIMU.angularVelocity.getZ());

      return averagedIMU[0].getHasBufferWindowFilled();
   }

   @Override
   public String getName()
   {
      return name;
   }
}
