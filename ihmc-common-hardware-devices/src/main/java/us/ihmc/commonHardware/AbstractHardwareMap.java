package us.ihmc.commonHardware;

import us.ihmc.commonHardware.devices.MechanismManagerInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.cycloids.CycloidPlatinumTwitter;
import us.ihmc.commonHardware.devices.cycloids.YoCycloidPlatinumTwitter;
import us.ihmc.commonHardware.devices.etherCATDevices.elmo.PlatinumTwitter.TWITTER_PRODUCT_CODE;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.H4EtherCATJunctionPort;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.H4IMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.YoH4IMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksBoardInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksIMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksTemperatureSensor;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.YoTemperatureSensor;
import us.ihmc.commonHardware.devices.genericIMU.GeneralIMUManager;
import us.ihmc.commonHardware.devices.genericIMU.IMUManagerInterface;
import us.ihmc.commonHardware.devices.genericIMU.YoGenericIMU;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.commonHardware.xmlDescription.XmlHardwareDescription;
import us.ihmc.commonHardware.xmlDescription.devices.XmlDevices;
import us.ihmc.commonHardware.xmlDescription.devices.XmlH4EtherCATJunctionPort;
import us.ihmc.commonHardware.xmlDescription.devices.XmlIMU;
import us.ihmc.commonHardware.xmlDescription.devices.XmlIMUType;
import us.ihmc.commonHardware.xmlDescription.devices.XmlPlatinumTwitter;
import us.ihmc.commonHardware.xmlDescription.devices.XmlTemperatureSensor;
import us.ihmc.commonHardware.xmlDescription.joints.XmlJoints;
import us.ihmc.commonHardware.xmlDescription.transmissions.XmlTransmissions;
import us.ihmc.commons.lists.PairList;
import us.ihmc.etherCAT.master.MasterInterface;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public abstract class AbstractHardwareMap
{
   protected final YoRegistry registry = new YoRegistry("HardwareMap");
   protected final YoGraphicsListRegistry yoGraphicsListRegistry;
   protected final MasterInterface etherCATMaster;
   protected final double dt;
   protected final YoDouble yoTime;

   protected final ArrayList<Slave> etherCATDevices = new ArrayList<>();

   protected final ArrayList<IMUManagerInterface> imuManagers = new ArrayList<>();
   protected final PairList<String, ImuData> measuredIMUData = new PairList<>();
   protected final Map<String, IMUDefinition> imuDefinitions = new HashMap<>();

   protected final ArrayList<YoCycloidPlatinumTwitter> cycloidTwitters = new ArrayList<>();
   protected final Map<String, YoCycloidPlatinumTwitter> cycloidPlatinumTwitterMap = new HashMap<>();
   protected final ArrayList<MechanismManagerInterface> mechanismManagers = new ArrayList<>();
   protected final YoBoolean doCycloidPDControlOnTwitters = new YoBoolean("doCycloidPDControlOnTwitters", registry);

   protected final String[] jointNames;
   protected final PairList<String, LowLevelState> measuredJointData = new PairList<>();
   protected final Map<String, JointDesiredOutputBasics> desiredJointData = new HashMap<>();

   protected final ArrayList<EtherSnacksBoardInterface> etherSnacksBoards = new ArrayList<>();
   protected final ArrayList<YoSensorInterface> yoEtherSnacksSensors = new ArrayList<>();

   protected Map<String, DoubleProvider> temperatureProviders = new HashMap<>();

   protected final HardwareStatusManager hardwareStatusManager = new HardwareStatusManager(registry);

   public AbstractHardwareMap(Collection<XmlHardwareDescription> xmlHardwareDescriptions,
                              MasterInterface etherCATMaster,
                              double dt,
                              YoDouble yoTime,
                              YoRegistry parentRegistry,
                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoTime = yoTime;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.dt = dt;
      this.etherCATMaster = etherCATMaster;

      for (XmlHardwareDescription xmlHardwareDescription : xmlHardwareDescriptions)
      {
         // Hardware maps will be skipped if they don't contain both devices and transmissions
         if (xmlHardwareDescription.getDevices() == null || xmlHardwareDescription.getTransmissions() == null)
            continue;

         XmlDevices devices = xmlHardwareDescription.getDevices();
         createDevices(devices);

         XmlTransmissions transmissions = xmlHardwareDescription.getTransmissions();
         createTransmissions(transmissions); // consider passing in devices here

         XmlJoints joints = xmlHardwareDescription.getJoints();
         if (joints != null)
            createJoints(joints);
      }

      jointNames = desiredJointData.keySet().toArray(new String[0]);

      parentRegistry.addChild(registry);
   }

   /**
    * Create instances of all the devices provided in the xml
    * @param devices XmlDevices object containing all the devicesin xml format
    */
   protected abstract void createDevices(XmlDevices devices);

   /**
    * Create instances of all the joints provided in the xml
    * @param joints XmlJoints object containing all the joints in xml format
    */
   protected abstract void createJoints(XmlJoints joints);

   /**
    * Create instances of all the transmissions provided in the xml
    * @param transmissions XmlTransmissions object containing all the joints in xml format
    */
   protected abstract void createTransmissions(XmlTransmissions transmissions);

   /**
    * Use to check if any of the objects created are null, which means there is a mistake in the xml
    * @param o object to check
    * @param msg message indicating which item is incorrect
    */
   protected void nullCheck(Object o, String msg)
   {
      if (o == null)
      {
         LogTools.fatal(msg);
         throw new NullPointerException(msg);
      }
   }

   /**
    * Create the H4 ethercat junction port objects and register them on the etherCAT line
    * @param xmlH4JunctionPort specific port to be initialized from xml
    */
   protected void createH4EtherCATJunctionPort(XmlH4EtherCATJunctionPort xmlH4JunctionPort)
   {
      int alias = xmlH4JunctionPort.getAlias();
      int position = xmlH4JunctionPort.getPosition();
      int junctionPort = xmlH4JunctionPort.getJunctionPort();
      H4EtherCATJunctionPort junction = new H4EtherCATJunctionPort(alias, position, junctionPort);
      etherCATMaster.registerSlave(junction);
      etherCATDevices.add(junction);
      hardwareStatusManager.registerDevice(xmlH4JunctionPort, junction);
   }

   /**
    * Create the IMU object and register on the etherCAT line
    * @param xmlIMU IMU to be initialized from xml
    */
   protected void createIMU(XmlIMU xmlIMU)
   {
      String name = xmlIMU.getName();
      XmlIMUType type = xmlIMU.getIMUType();
      int alias = xmlIMU.getAlias();
      int position = xmlIMU.getPosition();

      double angularBiasX = xmlIMU.getAngularBiasX();
      double angularBiasY = xmlIMU.getAngularBiasY();
      double angularBiasZ = xmlIMU.getAngularBiasZ();

      double linearBiasX = xmlIMU.getLinearBiasX();
      double linearBiasY = xmlIMU.getLinearBiasY();
      double linearBiasZ = xmlIMU.getLinearBiasZ();

      if (type == XmlIMUType.H4)
      {
         H4IMU imu = new H4IMU(alias, position);
         YoH4IMU yoImu = new YoH4IMU(name, imu, registry);

         yoImu.setLinearAccelerationBias(linearBiasX, linearBiasY, linearBiasZ);
         yoImu.setAngularVelocityBias(angularBiasX, angularBiasY, angularBiasZ);

         GeneralIMUManager imuManager = new GeneralIMUManager(imuDefinitions.get(name), yoImu, dt, registry);

         System.out.println("Registering " + name + " on " + alias + ":" + position);

         etherCATMaster.registerSlave(imu);
         etherCATDevices.add(imu);
         hardwareStatusManager.registerDevice(xmlIMU, imu);
         imuManagers.add(imuManager);
         measuredIMUData.add(imuManager.getName(), new ImuData());
      }
   }

   /**
    * Create the platinum twitter object and register on the etherCAT line
    * @param xmlPlatinumTwitter platinum twitter to be initialized from xml
    * @param parameterDirectory directory where the actuator package parameters are stored
    */
   protected void createPlatinumTwitter(XmlPlatinumTwitter xmlPlatinumTwitter, String parameterDirectory)
   {
      String name = xmlPlatinumTwitter.getName();
      int alias = xmlPlatinumTwitter.getAlias();
      int position = xmlPlatinumTwitter.getPosition();
      String actuatorPackage = xmlPlatinumTwitter.getActuatorPackage();
      boolean reverseMotorDirection = xmlPlatinumTwitter.isMotorDirectionReversed();
      double zeroPositionOffset = xmlPlatinumTwitter.getZeroPositionOffset();

      CycloidPlatinumTwitter cycloidPlatinumTwitter = new CycloidPlatinumTwitter(alias, position, TWITTER_PRODUCT_CODE.X00100002);
      YoCycloidPlatinumTwitter yoCycloidPlatinumTwitter = new YoCycloidPlatinumTwitter(name,
                                                                                       cycloidPlatinumTwitter,
                                                                                       yoTime,
                                                                                       parameterDirectory,
                                                                                       actuatorPackage,
                                                                                       reverseMotorDirection,
                                                                                       zeroPositionOffset,
                                                                                       dt,
                                                                                       registry);

      System.out.println("Registering " + name + " on " + alias + ":" + position);
      etherCATMaster.registerSlave(cycloidPlatinumTwitter);
      etherCATDevices.add(cycloidPlatinumTwitter);
      cycloidTwitters.add(yoCycloidPlatinumTwitter);
      cycloidPlatinumTwitterMap.put(name, yoCycloidPlatinumTwitter);
      hardwareStatusManager.registerDevice(xmlPlatinumTwitter, cycloidPlatinumTwitter);
   }

   /**
    * Create temperature sensor for an ethersnacks board
    * @param temperatureSensor temperature sensor to be initialized
    * @param parentName name of the parent board
    * @return ethersnacks temperature sensor object
    */
   protected EtherSnacksTemperatureSensor createEtherSnacksTemperatureSensor(XmlTemperatureSensor temperatureSensor, String parentName)
   {
      String name = temperatureSensor.getName();
      double scale = temperatureSensor.getConversionScale();
      double offset = temperatureSensor.getConversionOffset();

      EtherSnacksTemperatureSensor sensor = new EtherSnacksTemperatureSensor(scale, offset);
      YoTemperatureSensor yoSensor = new YoTemperatureSensor(name, sensor, registry);

      temperatureProviders.put(name, yoSensor.getTemperature());
      yoEtherSnacksSensors.add(yoSensor);
      return sensor;
   }

   /**
    * Create IMU for an ethersnacks board
    * @param xmlIMU IMU to be initialized
    * @param parentName name of the parent board
    * @return ethersnacks IMU object
    */
   protected EtherSnacksIMU createEtherSnacksIMU(XmlIMU xmlIMU, String parentName)
   {
      String name = xmlIMU.getName();

      double angularBiasX = xmlIMU.getAngularBiasX();
      double angularBiasY = xmlIMU.getAngularBiasY();
      double angularBiasZ = xmlIMU.getAngularBiasZ();

      double linearBiasX = xmlIMU.getLinearBiasX();
      double linearBiasY = xmlIMU.getLinearBiasY();
      double linearBiasZ = xmlIMU.getLinearBiasZ();

      EtherSnacksIMU imu = new EtherSnacksIMU(name);
      YoGenericIMU yoImu = new YoGenericIMU(name, imu, registry);

      yoImu.setAngularVelocityBias(angularBiasX, angularBiasY, angularBiasZ);
      yoImu.setLinearAccelerationBias(linearBiasX, linearBiasY, linearBiasZ);

      GeneralIMUManager imuManager = new GeneralIMUManager(imuDefinitions.get(name), yoImu, dt, registry);

      imuManagers.add(imuManager);
      measuredIMUData.add(imuManager.getName(), new ImuData());
      yoEtherSnacksSensors.add(yoImu);
      return imu;
   }

   public Slave[] getEtherCATDevices()
   {
      return etherCATDevices.toArray(new Slave[0]);
   }

   public IMUManagerInterface[] getImuManagers()
   {
      return imuManagers.toArray(new IMUManagerInterface[0]);
   }

   public PairList<String, ImuData> getMeasuredImuData()
   {
      return measuredIMUData;
   }

   public YoCycloidPlatinumTwitter[] getCycloidActuators()
   {
      return cycloidTwitters.toArray(new YoCycloidPlatinumTwitter[0]);
   }

   public EtherSnacksBoardInterface[] getEtherSnacksBoards()
   {
      return etherSnacksBoards.toArray(new EtherSnacksBoardInterface[0]);
   }

   public YoSensorInterface[] getYoEtherSnacksSensors()
   {
      return yoEtherSnacksSensors.toArray(new YoSensorInterface[0]);
   }

   public MechanismManagerInterface[] getMechanismManagers()
   {
      return mechanismManagers.toArray(new MechanismManagerInterface[0]);
   }

   public String[] getJointNames()
   {
      return jointNames;
   }

   public PairList<String, LowLevelState> getMeasuredJointData()
   {
      return measuredJointData;
   }

   public Map<String, JointDesiredOutputBasics> getDesiredJointData()
   {
      return desiredJointData;
   }

   public HardwareStatusManager getHardwareStatusManager()
   {
      return hardwareStatusManager;
   }
}
