package us.ihmc.commonHardware;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonHardware.devices.genericSensor.ForceSensorManagerInterface;
import us.ihmc.commonHardware.mechanisms.MechanismManagerInterface;
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
import us.ihmc.commonHardware.devices.genericSensor.GenericIMUManager;
import us.ihmc.commonHardware.devices.genericSensor.IMUManagerInterface;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericEncoder;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericIMU;
import us.ihmc.commonHardware.devices.genericSensor.YoGenericLoadCell;
import us.ihmc.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.hardwareXMLToolkit.XmlHardwareDescription;
import us.ihmc.hardwareXMLToolkit.XmlHardwareDescriptionLoader;
import us.ihmc.hardwareXMLToolkit.devices.XmlDevices;
import us.ihmc.hardwareXMLToolkit.devices.XmlEncoder;
import us.ihmc.hardwareXMLToolkit.devices.XmlH4EtherCATJunctionPort;
import us.ihmc.hardwareXMLToolkit.devices.XmlIMU;
import us.ihmc.hardwareXMLToolkit.devices.XmlIMUType;
import us.ihmc.hardwareXMLToolkit.devices.XmlLoadCell;
import us.ihmc.hardwareXMLToolkit.devices.XmlPlatinumTwitter;
import us.ihmc.hardwareXMLToolkit.devices.XmlTemperatureSensor;
import us.ihmc.hardwareXMLToolkit.joints.XmlJoints;
import us.ihmc.hardwareXMLToolkit.transmissions.XmlCycloidMechanism;
import us.ihmc.hardwareXMLToolkit.transmissions.XmlTransmissions;
import us.ihmc.commonHardware.mechanisms.CycloidMechanismManager;
import us.ihmc.etherCAT.master.MasterInterface;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.outputData.JointDesiredOutput;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class provides abstract structure for the hardware map of any robot we develop. It creates
 * the devices, transmissions, and joints that describe the robot based on the provided xml files.
 *
 * @author Reese Peterson
 */
public abstract class AbstractHardwareMap
{
   protected final YoRegistry registry = new YoRegistry("HardwareMap");
   protected final MasterInterface etherCATMaster;
   protected final double dt;
   protected final YoDouble yoTime;

   protected final Collection<XmlHardwareDescription> xmlHardwareDescriptions;
   protected final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   protected final ArrayList<Slave> etherCATDevices = new ArrayList<>();

   protected final ArrayList<String> imuNames = new ArrayList<>();
   protected final ArrayList<IMUManagerInterface> imuManagers = new ArrayList<>();
   protected final Map<String, ImuData> measuredIMUData = new HashMap<>();
   protected final Map<String, IMUDefinition> imuDefinitions = new HashMap<>();

   protected final ArrayList<String> forceSensorNames = new ArrayList<>();
   protected final ArrayList<ForceSensorManagerInterface> forceSensorManagers = new ArrayList<>();
   protected final Map<String, DMatrixRMaj> forceSensorData = new HashMap<>();

   protected final ArrayList<YoCycloidPlatinumTwitter> cycloidTwitters = new ArrayList<>();
   protected final Map<String, YoCycloidPlatinumTwitter> cycloidPlatinumTwitterMap = new HashMap<>();
   protected final ArrayList<MechanismManagerInterface> mechanismManagers = new ArrayList<>();
   protected final YoBoolean doCycloidPDControlOnTwitters = new YoBoolean("masterDoCycloidPDControlOnTwitters", registry);

   protected final ArrayList<String> jointNames = new ArrayList<>();
   protected final Map<String, LowLevelState> measuredJointData = new HashMap<>();
   protected final Map<String, JointDesiredOutputBasics> desiredJointData = new HashMap<>();

   protected final ArrayList<EtherSnacksBoardInterface> etherSnacksBoards = new ArrayList<>();
   protected final ArrayList<YoSensorInterface> yoEtherSnacksSensors = new ArrayList<>();

   protected Map<String, DoubleProvider> temperatureProviders = new HashMap<>();

   protected final HardwareStatusManager hardwareStatusManager = new HardwareStatusManager(registry);

   // Robot model resource stuff
   protected static final String XML_SUB_DIRECTORY = "hardware";
   protected static final String URDF_SUB_DIRECTORY = "urdf";
   protected static final String MESH_SUB_DIRECTORY = "meshes";

   /**
    * Construct the hardware map for the robot
    *
    * @param robotModelResourcesDirectory Directory containing robot model urdf and xml resources
    * @param xmlFiles                     xml files that make up xml description of the robot
    * @param urdfFiles                    urdf files that make up urdf description of the robot
    * @param etherCATMaster               Main ethercat device used to register all EtherCAT devices in the robot
    * @param dt                           desired control timesteo
    * @param yoTime                       YoDouble that holds the current time of the robot
    * @param parentRegistry               Parent YoRegistry
    */
   public AbstractHardwareMap(String robotModelResourcesDirectory,
                              List<String> xmlFiles,
                              List<String> urdfFiles,
                              MasterInterface etherCATMaster,
                              double dt,
                              YoDouble yoTime,
                              YoRegistry parentRegistry)
   {
      this(XmlHardwareDescriptionLoader.getHardwareDescriptionFromAlternateResources(robotModelResourcesDirectory + XML_SUB_DIRECTORY + '/', xmlFiles),
           List.of(robotModelResourcesDirectory,
                   robotModelResourcesDirectory + URDF_SUB_DIRECTORY + '/',
                   robotModelResourcesDirectory + MESH_SUB_DIRECTORY + '/'),
           urdfFiles.stream().map(file ->
                                  {
                                     if (file.contains("ezGripper/") || file.contains("abilityHand/"))
                                        return file;
                                     else
                                        return robotModelResourcesDirectory + URDF_SUB_DIRECTORY + '/' + file;
                                  }).toList(),
           etherCATMaster,
           null,
           dt,
           yoTime,
           parentRegistry);
   }

   /**
    * Construct the hardware map for the robot
    *
    * @param xmlHardwareDescriptions         Collection of XmlHardwareDescriptions that contain the necessary devices, joints, and transmissions
    * @param etherCATMaster                  Main ethercat device used to register all EtherCAT devices in the robot
    * @param stateEstimatorSensorDefinitions Sensor definitions for the state estimator. If there is no state estimator, then leave as null
    * @param dt                              desired control timesteo
    * @param yoTime                          YoDouble that holds the current time of the robot
    * @param parentRegistry                  Parent YoRegistry
    */
   public AbstractHardwareMap(Collection<XmlHardwareDescription> xmlHardwareDescriptions,
                              List<String> urdfResourceDirectories,
                              List<String> urdfResources,
                              MasterInterface etherCATMaster,
                              @Nullable StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                              double dt,
                              YoDouble yoTime,
                              YoRegistry parentRegistry)
   {
      this.xmlHardwareDescriptions = xmlHardwareDescriptions;
      this.stateEstimatorSensorDefinitions = stateEstimatorSensorDefinitions;
      this.yoTime = yoTime;
      this.dt = dt;
      this.etherCATMaster = etherCATMaster;

      createSensorDefinitions(stateEstimatorSensorDefinitions, urdfResources, urdfResourceDirectories);

      parentRegistry.addChild(registry);
   }

   protected void createXmlDefinitions()
   {
      for (XmlHardwareDescription xmlHardwareDescription : xmlHardwareDescriptions)
      {
         if (xmlHardwareDescription.getDevices() != null)
         {
            XmlDevices devices = xmlHardwareDescription.getDevices();
            createDevices(devices);
         }

         if (xmlHardwareDescription.getTransmissions() != null)
         {
            XmlTransmissions transmissions = xmlHardwareDescription.getTransmissions();
            createTransmissions(transmissions);
         }

         if (xmlHardwareDescription.getJoints() != null)
         {
            XmlJoints joints = xmlHardwareDescription.getJoints();
            createJoints(joints);
         }
      }

      jointNames.clear();
      jointNames.addAll(measuredJointData.keySet());

      imuNames.clear();
      imuNames.addAll(measuredIMUData.keySet());

      forceSensorNames.clear();
      forceSensorNames.addAll(forceSensorData.keySet());
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

   /**
    * Create the sensor definitions for all devices
    *
    * @param stateEstimatorSensorDefinitions If not null, use the definitions provided to create the sensor definitions
    */
   protected abstract void createSensorDefinitions(@Nullable StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, List<String> urdfResources, List<String> urdfResourceDirectories);

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
      int vendorID = xmlIMU.getVendorID();
      int productCode = xmlIMU.getProductCode();

      double angularBiasX = xmlIMU.getAngularVelocityBiasX();
      double angularBiasY = xmlIMU.getAngularVelocityBiasY();
      double angularBiasZ = xmlIMU.getAngularVelocityBiasZ();

      double linearBiasX = xmlIMU.getLinearAccelerationBiasX();
      double linearBiasY = xmlIMU.getLinearAccelerationBiasY();
      double linearBiasZ = xmlIMU.getLinearAccelerationBiasZ();

      if (type == XmlIMUType.H4)
      {
         H4IMU imu = new H4IMU(vendorID, productCode, alias, position);
         YoH4IMU yoImu = new YoH4IMU(name, imu, registry);

         yoImu.setLinearAccelerationBias(linearBiasX, linearBiasY, linearBiasZ);
         yoImu.setAngularVelocityBias(angularBiasX, angularBiasY, angularBiasZ);

         GenericIMUManager imuManager = new GenericIMUManager(imuDefinitions.get(name), yoImu, dt, registry);

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
      double motorOffset = xmlPlatinumTwitter.getMotorOffset();
      double outputOffset = xmlPlatinumTwitter.getOutputOffset();
      boolean dynamicBrakingEnabled = xmlPlatinumTwitter.isDynamicBrakingEnabled();
      boolean outputFromMotorEncoder = xmlPlatinumTwitter.getOutputFromMotorEncoder();

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
                                                                                       motorOffset,
                                                                                       outputOffset,
                                                                                       dt,
                                                                                       dynamicBrakingEnabled,
                                                                                       false,
                                                                                       registry);

      yoCycloidPlatinumTwitter.setOutputEncoderInverted(xmlPlatinumTwitter.isOutputEncoderInverted());
      yoCycloidPlatinumTwitter.setUseOutputPositionFromMotor(outputFromMotorEncoder);
      yoCycloidPlatinumTwitter.setUseOutputVelocityFromMotor(outputFromMotorEncoder);
      System.out.println("Registering " + name + " on " + alias + ":" + position);
      etherCATMaster.registerSlave(cycloidPlatinumTwitter);
      etherCATDevices.add(cycloidPlatinumTwitter);
      cycloidTwitters.add(yoCycloidPlatinumTwitter);
      cycloidPlatinumTwitterMap.put(name, yoCycloidPlatinumTwitter);
      hardwareStatusManager.registerDevice(xmlPlatinumTwitter, yoCycloidPlatinumTwitter);
   }

   /**
    * Create the cycloid mechanism manager described in the xml
    *
    * @param mechanism mechanism information from the xmls
    */
   protected void createCycloidMechanismManager(XmlCycloidMechanism mechanism)
   {
      String jointName = mechanism.getJointName();
      String motorName = mechanism.getMotorName();
      double jointOffset = mechanism.getJointPositionOffset();
      double upperLimit = mechanism.getUpperJointLimit();
      double lowerLimit = mechanism.getLowerJointLimit();
      double torqueBreakFrequency = mechanism.getTorqueBreakFrequency();
      boolean useFilteredStates = mechanism.useFilteredStates();
      boolean publishFilteredStates = mechanism.publishFilteredStates();
      boolean doPDControlOnTwitter = mechanism.doPDControlOnTwitter();

      CycloidMechanismManager cycloidMechanismManager = createCycloidMechanismManager(jointName,
                                                                                      motorName,
                                                                                      jointOffset,
                                                                                      lowerLimit,
                                                                                      upperLimit,
                                                                                      torqueBreakFrequency,
                                                                                      useFilteredStates,
                                                                                      publishFilteredStates);
      cycloidMechanismManager.doPDControlOnTwitter(doPDControlOnTwitter);
      mechanismManagers.add(cycloidMechanismManager);
      measuredJointData.put(cycloidMechanismManager.getName(), new LowLevelState(0.0, 0.0, 0.0, 0.0));
      desiredJointData.put(cycloidMechanismManager.getName(), new JointDesiredOutput());
   }

   protected CycloidMechanismManager createCycloidMechanismManager(String jointName,
                                                                   String motorName,
                                                                   double jointOffset,
                                                                   double jointLimitLower,
                                                                   double jointLimitUpper,
                                                                   double torqueBreakFrequency,
                                                                   boolean useFilteredStates,
                                                                   boolean publishFilteredStates)
   {
      YoCycloidPlatinumTwitter platinumTwitter = cycloidPlatinumTwitterMap.get(motorName);
      nullCheck(platinumTwitter, motorName + " Not found, Likely incorrect name in XML Hardware Description");

      return new CycloidMechanismManager(jointOffset,
                                         jointLimitLower,
                                         jointLimitUpper,
                                         jointName,
                                         platinumTwitter,
                                         yoTime,
                                         this.dt,
                                         doCycloidPDControlOnTwitters,
                                         torqueBreakFrequency,
                                         useFilteredStates,
                                         publishFilteredStates,
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

      if (xmlIMU.isPresent())
      {
         GenericIMUManager imuManager = new GenericIMUManager(imuDefinitions.get(name), yoImu, dt, registry);

         imuManagers.add(imuManager);
         measuredIMUData.put(imuManager.getName(), new ImuData());
      }

      yoEtherSnacksSensors.add(yoImu);
      return imu;
   }

   /**
    * Create encoder for an ethersnacks board
    *
    * @param xmlEncoder Encoder to be initialized
    * @param parentName name of the parent board
    * @return ethersnacks IMU object
    */
   protected EtherSnacksEncoder createEtherSnacksEncoder(XmlEncoder xmlEncoder, String parentName)
   {
      String name = xmlEncoder.getName();

      EtherSnacksEncoder encoder = new EtherSnacksEncoder(name);
      YoGenericEncoder yoEncoder = new YoGenericEncoder(name, encoder, xmlEncoder.isInvertDirection(), dt, registry);

      yoEtherSnacksSensors.add(yoEncoder);

      return encoder;
   }

   /**
    * Create load cell for an ethersnacks board
    *
    * @param xmlLoadCell Load cell to be initialized
    * @param parentName  name of the parent board
    * @return ethersnacks IMU object
    */
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

   /**
    * @return array of ethercat devices
    */
   public Slave[] getEtherCATDevices()
   {
      return etherCATDevices.toArray(new Slave[0]);
   }

   /**
    * @return array of all IMU managers
    */
   public IMUManagerInterface[] getImuManagers()
   {
      return imuManagers.toArray(new IMUManagerInterface[0]);
   }

   /**
    * @return array of all cycloid platinum twitters
    */
   public YoCycloidPlatinumTwitter[] getCycloidTwitters()
   {
      return cycloidTwitters.toArray(new YoCycloidPlatinumTwitter[0]);
   }

   /**
    * @return array of all ethersnacks boards
    */
   public EtherSnacksBoardInterface[] getEtherSnacksBoards()
   {
      return etherSnacksBoards.toArray(new EtherSnacksBoardInterface[0]);
   }

   /**
    * @return array of all ethersnacks sensors
    */
   public YoSensorInterface[] getYoEtherSnacksSensors()
   {
      return yoEtherSnacksSensors.toArray(new YoSensorInterface[0]);
   }

   /**
    * @return array of all mechanism managers
    */
   public MechanismManagerInterface[] getMechanismManagers()
   {
      return mechanismManagers.toArray(new MechanismManagerInterface[0]);
   }

   /**
    * @return array of joint names as strings
    */
   public String[] getJointNames()
   {
      return jointNames.toArray(new String[0]);
   }

   /**
    * @return array of IMU sensor names as strings
    */
   public String[] getIMUNames()
   {
      return imuNames.toArray(new String[0]);
   }

   /**
    * @return array of force sensor names as strings
    */
   public String[] getForceSensorNames()
   {
      return forceSensorNames.toArray(new String[0]);
   }

   /**
    * @return array of all force sensor managers
    */
   public ForceSensorManagerInterface[] getForceSensorManagers()
   {
      return forceSensorManagers.toArray(new ForceSensorManagerInterface[0]);
   }

   /**
    * @return map tying measured IMU data to the specific imu sensor name
    */
   public Map<String, ImuData> getMeasuredImuData()
   {
      return measuredIMUData;
   }

   /**
    * @return map tying force sensor data to the respective force sensor name
    */
   public Map<String, DMatrixRMaj> getMeasuredForceSensorData()
   {
      return forceSensorData;
   }

   /**
    * @return map tying measured joint data to the respective joint name
    */
   public Map<String, LowLevelState> getMeasuredJointData()
   {
      return measuredJointData;
   }

   /**
    * @return map tying desired joint data to the respective joint name
    */
   public Map<String, JointDesiredOutputBasics> getDesiredJointData()
   {
      return desiredJointData;
   }

   /**
    * @return hardware status manager for the robot
    */
   public HardwareStatusManager getHardwareStatusManager()
   {
      return hardwareStatusManager;
   }
}
