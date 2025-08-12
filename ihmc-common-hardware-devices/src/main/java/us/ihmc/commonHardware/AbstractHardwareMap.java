package us.ihmc.commonHardware;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonHardware.devices.ForceSensorManagerInterface;
import us.ihmc.commonHardware.devices.MechanismManagerInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.cycloids.CycloidPlatinumTwitter;
import us.ihmc.commonHardware.devices.cycloids.YoCycloidPlatinumTwitter;
import us.ihmc.commonHardware.devices.etherCATDevices.elmo.PlatinumTwitter.TWITTER_PRODUCT_CODE;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.H4EtherCATJunctionPort;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.H4IMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.YoH4IMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksBoardInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksEncoder;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksIMU;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksLoadCell;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksTemperatureSensor;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.YoTemperatureSensor;
import us.ihmc.commonHardware.devices.genericSensor.GeneralIMUManager;
import us.ihmc.commonHardware.devices.genericSensor.IMUManagerInterface;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericEncoder;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericIMU;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericLoadCell;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.commonHardware.mechanisms.CycloidMotorMechanismManager;
import us.ihmc.commonHardware.xmlDescription.XmlHardwareDescription;
import us.ihmc.commonHardware.xmlDescription.devices.XmlDevices;
import us.ihmc.commonHardware.xmlDescription.devices.XmlEncoder;
import us.ihmc.commonHardware.xmlDescription.devices.XmlH4EtherCATJunctionPort;
import us.ihmc.commonHardware.xmlDescription.devices.XmlIMU;
import us.ihmc.commonHardware.xmlDescription.devices.XmlIMUType;
import us.ihmc.commonHardware.xmlDescription.devices.XmlLoadCell;
import us.ihmc.commonHardware.xmlDescription.devices.XmlPlatinumTwitter;
import us.ihmc.commonHardware.xmlDescription.devices.XmlTemperatureSensor;
import us.ihmc.commonHardware.xmlDescription.joints.XmlJoints;
import us.ihmc.commonHardware.xmlDescription.transmissions.XmlCycloidMotorMechanism;
import us.ihmc.commonHardware.xmlDescription.transmissions.XmlTransmissions;
import us.ihmc.etherCAT.master.MasterInterface;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.outputData.JointDesiredOutput;
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

   protected final String[] imuNames;
   protected final ArrayList<IMUManagerInterface> imuManagers = new ArrayList<>();
   protected final Map<String, ImuData> measuredIMUData = new HashMap<>();
   protected final Map<String, IMUDefinition> imuDefinitions = new HashMap<>();

   protected final String[] forceSensorNames;
   protected final ArrayList<ForceSensorManagerInterface> forceSensorManagers = new ArrayList<>();
   protected final Map<String, DMatrixRMaj> forceSensorData = new HashMap<>();

   protected final ArrayList<YoCycloidPlatinumTwitter> cycloidTwitters = new ArrayList<>();
   protected final Map<String, YoCycloidPlatinumTwitter> cycloidPlatinumTwitterMap = new HashMap<>();
   protected final ArrayList<MechanismManagerInterface> mechanismManagers = new ArrayList<>();
   protected final YoBoolean doCycloidPDControlOnTwitters = new YoBoolean("doCycloidPDControlOnTwitters", registry);

   protected final String[] jointNames;
   protected final Map<String, LowLevelState> measuredJointData = new HashMap<>();
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

      createSensorDefinitions();

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

      jointNames = measuredJointData.keySet().toArray(new String[0]);
      imuNames = measuredIMUData.keySet().toArray(new String[0]);
      forceSensorNames = forceSensorData.keySet().toArray(new String[0]);

      parentRegistry.addChild(registry);
   }

   /**
    * Create instances of all the devices provided in the xml
    *
    * @param devices XmlDevices object containing all the devicesin xml format
    */
   protected abstract void createDevices(XmlDevices devices);

   /**
    * Create instances of all the joints provided in the xml
    *
    * @param joints XmlJoints object containing all the joints in xml format
    */
   protected abstract void createJoints(XmlJoints joints);

   /**
    * Create instances of all the transmissions provided in the xml
    *
    * @param transmissions XmlTransmissions object containing all the joints in xml format
    */
   protected abstract void createTransmissions(XmlTransmissions transmissions);

   /**
    * Use to check if any of the objects created are null, which means there is a mistake in the xml
    *
    * @param o   object to check
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

   protected abstract void createSensorDefinitions();

   /**
    * Create the H4 ethercat junction port objects and register them on the etherCAT line
    *
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
    *
    * @param xmlIMU IMU to be initialized from xml
    */
   protected void createIMU(XmlIMU xmlIMU)
   {
      String name = xmlIMU.getName();
      XmlIMUType type = xmlIMU.getIMUType();
      int alias = xmlIMU.getAlias();
      int position = xmlIMU.getPosition();

      double angularBiasX = xmlIMU.getAngularVelocityBiasX();
      double angularBiasY = xmlIMU.getAngularVelocityBiasY();
      double angularBiasZ = xmlIMU.getAngularVelocityBiasZ();

      double linearBiasX = xmlIMU.getLinearAccelerationBiasX();
      double linearBiasY = xmlIMU.getLinearAccelerationBiasY();
      double linearBiasZ = xmlIMU.getLinearAccelerationBiasZ();

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
         measuredIMUData.put(imuManager.getName(), new ImuData());
      }
   }

   /**
    * Create the platinum twitter object and register on the etherCAT line
    *
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
      int inputOffset = xmlPlatinumTwitter.getInputOffset();
      int outputOffset = xmlPlatinumTwitter.getOutputOffset();

      CycloidPlatinumTwitter cycloidPlatinumTwitter;
      if(xmlPlatinumTwitter.useLatestCode())
         cycloidPlatinumTwitter = new CycloidPlatinumTwitter(alias, position);
      else
         cycloidPlatinumTwitter = new CycloidPlatinumTwitter(alias, position, TWITTER_PRODUCT_CODE.X00100002);

      YoCycloidPlatinumTwitter yoCycloidPlatinumTwitter = new YoCycloidPlatinumTwitter(name,
                                                                                       cycloidPlatinumTwitter,
                                                                                       yoTime,
                                                                                       parameterDirectory,
                                                                                       actuatorPackage,
                                                                                       reverseMotorDirection,
                                                                                       inputOffset,
                                                                                       outputOffset,
                                                                                       dt,
                                                                                       registry);

      System.out.println("Registering " + name + " on " + alias + ":" + position);
      etherCATMaster.registerSlave(cycloidPlatinumTwitter);
      etherCATDevices.add(cycloidPlatinumTwitter);
      cycloidTwitters.add(yoCycloidPlatinumTwitter);
      cycloidPlatinumTwitterMap.put(name, yoCycloidPlatinumTwitter);
      hardwareStatusManager.registerDevice(xmlPlatinumTwitter, cycloidPlatinumTwitter);
   }

   protected void createCycloidMechanismManager(XmlCycloidMotorMechanism mechanism)
   {
      String jointName = mechanism.getJointName();
      String motorName = mechanism.getMotorName();
      double jointOffset = mechanism.getJointPositionOffset();
      double upperLimit = mechanism.getUpperJointLimit();
      double lowerLimit = mechanism.getLowerJointLimit();
      double torqueBreakFrequency = mechanism.getTorqueBreakFrequency();

      CycloidMotorMechanismManager cycloidMotorMechanismManager = createCycloidMechanismManager(jointName,
                                                                                                motorName,
                                                                                                jointOffset,
                                                                                                lowerLimit,
                                                                                                upperLimit,
                                                                                                torqueBreakFrequency); //TODO add joint limits to xml
      mechanismManagers.add(cycloidMotorMechanismManager);
      measuredJointData.put(cycloidMotorMechanismManager.getName(), new LowLevelState(0.0, 0.0, 0.0, 0.0));
      desiredJointData.put(cycloidMotorMechanismManager.getName(), new JointDesiredOutput());
   }

   protected CycloidMotorMechanismManager createCycloidMechanismManager(String jointName,
                                                                        String motorName,
                                                                        double jointOffset,
                                                                        double jointLimitLower,
                                                                        double jointLimitUpper,
                                                                        double torqueBreakFrequency)
   {
      YoCycloidPlatinumTwitter platinumTwitter = cycloidPlatinumTwitterMap.get(motorName);
      nullCheck(platinumTwitter, motorName + " Not found, Likely incorrect name in XML Hardware Description");

      return new CycloidMotorMechanismManager(jointOffset,
                                              jointLimitLower,
                                              jointLimitUpper,
                                              jointName,
                                              platinumTwitter,
                                              yoTime,
                                              this.dt,
                                              doCycloidPDControlOnTwitters,
                                              torqueBreakFrequency,
                                              registry);
   }

   /**
    * Create temperature sensor for an ethersnacks board
    *
    * @param temperatureSensor temperature sensor to be initialized
    * @param parentName        name of the parent board
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
    *
    * @param xmlIMU     IMU to be initialized
    * @param parentName name of the parent board
    * @return ethersnacks IMU object
    */
   protected EtherSnacksIMU createEtherSnacksIMU(XmlIMU xmlIMU, String parentName)
   {
      String name = xmlIMU.getName();

      double angularBiasX = xmlIMU.getAngularVelocityBiasX();
      double angularBiasY = xmlIMU.getAngularVelocityBiasY();
      double angularBiasZ = xmlIMU.getAngularVelocityBiasZ();

      double linearBiasX = xmlIMU.getLinearAccelerationBiasX();
      double linearBiasY = xmlIMU.getLinearAccelerationBiasY();
      double linearBiasZ = xmlIMU.getLinearAccelerationBiasZ();

      EtherSnacksIMU imu = new EtherSnacksIMU(name);
      YoGenericIMU yoImu = new YoGenericIMU(name, imu, registry);

      yoImu.setAngularVelocityBias(angularBiasX, angularBiasY, angularBiasZ);
      yoImu.setLinearAccelerationBias(linearBiasX, linearBiasY, linearBiasZ);

      GeneralIMUManager imuManager = new GeneralIMUManager(imuDefinitions.get(name), yoImu, dt, registry);

      imuManagers.add(imuManager);
      measuredIMUData.put(imuManager.getName(), new ImuData());
      yoEtherSnacksSensors.add(yoImu);
      return imu;
   }

   protected EtherSnacksEncoder createEtherSnacksEncoder(XmlEncoder xmlEncoder, String parentName)
   {
      String name = xmlEncoder.getName();

      EtherSnacksEncoder encoder = new EtherSnacksEncoder(name);
      YoGenericEncoder yoEncoder = new YoGenericEncoder(name, encoder, xmlEncoder.isInvertDirection(), dt, registry);

      yoEtherSnacksSensors.add(yoEncoder);

      return encoder;
   }

   protected EtherSnacksLoadCell createEtherSnacksLoadCell(XmlLoadCell xmlLoadCell, String parentName)
   {
      String name = xmlLoadCell.getName();
      double excitationVoltage = xmlLoadCell.getExcitationVoltage();
      double nominalLoad = xmlLoadCell.getNominalLoad();
      double nominalSensitivity = xmlLoadCell.getNominalSensitivity();
      double zeroBalance = xmlLoadCell.getZeroBalance();

      EtherSnacksLoadCell loadCell = new EtherSnacksLoadCell(name);
      YoGenericLoadCell yoLoadCell = new YoGenericLoadCell(name, loadCell, nominalSensitivity, zeroBalance, nominalLoad, excitationVoltage, registry);

      yoEtherSnacksSensors.add(yoLoadCell);

      return loadCell;
   }

   public Slave[] getEtherCATDevices()
   {
      return etherCATDevices.toArray(new Slave[0]);
   }

   public IMUManagerInterface[] getImuManagers()
   {
      return imuManagers.toArray(new IMUManagerInterface[0]);
   }

   public Map<String, ImuData> getMeasuredImuData()
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

   public String[] getIMUNames()
   {
      return imuNames;
   }

   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   public ForceSensorManagerInterface[] getForceSensorManagers()
   {
      return forceSensorManagers.toArray(new ForceSensorManagerInterface[0]);
   }

   public Map<String, DMatrixRMaj> getMeasuredFTData()
   {
      return forceSensorData;
   }

   public Map<String, LowLevelState> getMeasuredJointData()
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
