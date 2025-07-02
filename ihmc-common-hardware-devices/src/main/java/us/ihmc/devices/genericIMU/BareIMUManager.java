package us.ihmc.devices.genericIMU;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredTuple3D;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Map;

public class BareIMUManager implements IMUManagerInterface
{
   /**
    * Useful for debugging the main IMU on the pelvis.
    */
   public static boolean COMPUTE_ROOT_JOINT_IMU = false;
   private final String name;
   private final YoGenericIMU yoIMU;

   private final YoBoolean filterIMUReadings;

   private final AlphaFilteredTuple3D filteredAngularVelocity;
   private final AlphaFilteredTuple3D filteredLinearAcceleration;
   private final YoVector3D yoFilteredAngularVelocity;
   private final YoVector3D yoFilteredLinearAcceleration;


   private final YoRegistry registry;

   public BareIMUManager(String name, YoGenericIMU yoIMU, YoRegistry parentRegistry)
   {
      this.name = name;
      this.yoIMU = yoIMU;

      registry = new YoRegistry(name + getClass().getSimpleName());

      filterIMUReadings = new YoBoolean(name + "filterH4IMUReadings", registry);

      YoDouble h4IMUAngularVelocityFilterAlpha = new YoDouble(name + "h4IMUAngularVelocityFilterAlpha", registry);
      YoDouble h4IMULinearAccelerationFilterAlpha = new YoDouble(name + "h4IMULinearAccelerationFilterAlpha", registry);

      filteredAngularVelocity = new AlphaFilteredTuple3D();
      filteredLinearAcceleration = new AlphaFilteredTuple3D();
      yoFilteredAngularVelocity = new YoVector3D(name, "filteredAngularVelocity", registry);
      yoFilteredLinearAcceleration = new YoVector3D(name, "filteredLinearAcceleration", registry);

      h4IMULinearAccelerationFilterAlpha.addListener(s -> filteredLinearAcceleration.setAlpha(MathTools.clamp(h4IMULinearAccelerationFilterAlpha.getValue(), 0.0, 1.0)));
      h4IMUAngularVelocityFilterAlpha.addListener(s -> filteredAngularVelocity.setAlpha(MathTools.clamp(h4IMUAngularVelocityFilterAlpha.getValue(), 0.0, 1.0)));

      parentRegistry.addChild(registry);
   }

   @Override
   public void read(Map<String, ImuData> measuredIMUDataMap)
   {
      yoIMU.update();

      ImuData measuredIMUData = measuredIMUDataMap.get(name);

      filteredAngularVelocity.set(yoIMU.getUnbiasedAngularVelocity());
      filteredLinearAcceleration.set(yoIMU.getUnbiasedLinearAcceleration());

      yoFilteredAngularVelocity.set(filteredAngularVelocity);
      yoFilteredLinearAcceleration.set(filteredLinearAcceleration);

      if (measuredIMUData != null)
      {
         if (filterIMUReadings.getBooleanValue())
         {
            measuredIMUData.setLinearAcceleration(yoFilteredLinearAcceleration);
            measuredIMUData.setAngularVelocity(yoFilteredAngularVelocity);
         }
         else
         {
            measuredIMUData.setLinearAcceleration(yoIMU.getUnbiasedLinearAcceleration());
            measuredIMUData.setAngularVelocity(yoIMU.getUnbiasedAngularVelocity());
         }
      }
   }

   @Override
   public String getName()
   {
      return name;
   }
}
