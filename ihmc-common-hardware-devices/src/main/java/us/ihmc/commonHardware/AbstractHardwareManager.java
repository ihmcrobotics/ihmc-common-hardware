package us.ihmc.commonHardware;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonHardware.devices.genericSensor.ForceSensorManagerInterface;
import us.ihmc.commonHardware.mechanisms.MechanismManagerInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksBoardInterface;
import us.ihmc.commonHardware.devices.genericSensor.IMUManagerInterface;
import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This class provides abstract structure for managing all the devices and mechanisms of a robot
 *
 * @author Reese Peterson
 */
public abstract class AbstractHardwareManager
{
   protected final YoRegistry registry;

   protected final MechanismManagerInterface[] mechanismManagers;
   protected final IMUManagerInterface[] imuManagers;
   protected final ForceSensorManagerInterface[] forceSensorManagers;
   protected final AtomicBoolean calibrateFootForceSensorsAtomic = new AtomicBoolean(false);

   protected final EtherSnacksBoardInterface[] etherSnacksBoards;
   protected final YoSensorInterface[] yoEtherSnacksSensors;

   protected final Slave[] etherCATDevices;

   protected final String[] jointNames;

   protected final DoubleProvider clockTime;

   protected final YoLong imuReadTime;
   protected final YoLong forceSensorReadTime;
   protected final YoLong mechanismReadTime;
   protected final YoLong etherSnacksReadTime;
   protected final YoLong readTime;

   protected final YoLong forceSensorWriteTime;
   protected final YoLong mechanismWriteTime;
   protected final YoLong etherSnacksWriteTime;
   protected final YoLong writeTime;

   protected final YoBoolean areMotorsFaulted;

   protected final YoDouble masterGain;

   protected final HardwareStatusManager hardwareStatusManager;

   protected final YoBoolean isMotorWarm;
   protected final YoBoolean hasMotorOverHeated;
   protected final YoDouble totalMeasuredMotorCurrent;

   protected final YoDouble mainActuatorPositionBreakFrequency;
   protected final YoDouble mainActuatorVelocityBreakFrequency;
   protected final YoBoolean useMainActuatorBreakFrequencies;

   protected final YoDouble mainMotorPositionBreakFrequency;
   protected final YoDouble mainMotorVelocityBreakFrequency;
   protected final YoBoolean useMainMotorBreakFrequencies;

   protected RobotOverHeatedListener robotOverHeatedListener;

   /**
    * Constructs the hardware manager for the robot
    * @param hardwareMap Contains objects to communicate with all devices and mechanisms
    * @param clockTime Clock time of the robot
    * @param parentRegistry Parent YoRegistry
    */
   public AbstractHardwareManager(AbstractHardwareMap hardwareMap, DoubleProvider clockTime, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(getClass().getSimpleName());

      this.clockTime = clockTime;
      mechanismManagers = hardwareMap.getMechanismManagers();
      imuManagers = hardwareMap.getImuManagers();
      forceSensorManagers = hardwareMap.getForceSensorManagers();
      etherCATDevices = hardwareMap.getEtherCATDevices();
      etherSnacksBoards = hardwareMap.getEtherSnacksBoards();
      yoEtherSnacksSensors = hardwareMap.getYoEtherSnacksSensors();
      hardwareStatusManager = hardwareMap.getHardwareStatusManager();

      imuReadTime = new YoLong("imuReadTime", registry);
      forceSensorReadTime = new YoLong("forceSensorReadTime", registry);
      mechanismReadTime = new YoLong("mechanismReadTime", registry);
      etherSnacksReadTime = new YoLong("etherSnacksReadTime", registry);
      readTime = new YoLong("readTime", registry);

      forceSensorWriteTime = new YoLong("forceSensorWriteTime", registry);
      mechanismWriteTime = new YoLong("mechanismWriteTime", registry);
      etherSnacksWriteTime = new YoLong("etherSnacksWriteTime", registry);
      writeTime = new YoLong("writeTime", registry);

      hasMotorOverHeated = new YoBoolean("hasArmMotorOverHeated", registry);
      isMotorWarm = new YoBoolean("isArmMotorWarm", registry);
      hasMotorOverHeated.addListener(s ->
      {
         if (robotOverHeatedListener != null)
            robotOverHeatedListener.changed(hasMotorOverHeated.getValue());
      });

      totalMeasuredMotorCurrent = new YoDouble("totalMeasuredMotorCurrent", registry);

      masterGain = new YoDouble("lowLevelMasterGain", registry);

      mainActuatorPositionBreakFrequency = new YoDouble("mainActuatorPositionBreakFrequency", registry);
      mainActuatorVelocityBreakFrequency = new YoDouble("mainActuatorVelocityBreakFrequency", registry);
      useMainActuatorBreakFrequencies = new YoBoolean("useMainBreakFrequencies", registry);

      mainMotorPositionBreakFrequency = new YoDouble("mainMotorPositionBreakFrequency", registry);
      mainMotorVelocityBreakFrequency = new YoDouble("mainMotorVelocityBreakFrequency", registry);
      useMainMotorBreakFrequencies = new YoBoolean("useMainMotorBreakFrequencies", registry);

      areMotorsFaulted = new YoBoolean("AreMotorsFaulted", registry);

      jointNames = hardwareMap.getJointNames();

      parentRegistry.addChild(registry);
   }

   /**
    * Reads all the data from the IMUs, force sensors, and joint sensors and stores them in their respective manager maps
    *
    * @param measuredIMUData   Map to store IMU data
    * @param measuredFTData    Map to store force and torque data
    * @param measuredJointData Map to store joint data
    */
   public void read(Map<String, ImuData> measuredIMUData, Map<String, DMatrixRMaj> measuredFTData, Map<String, LowLevelState> measuredJointData)
   {
      long readStartTime = RealtimeThread.getCurrentMonotonicClockTime();

      // Read the EtherSnacks boards
      long etherSnacksReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (EtherSnacksBoardInterface etherSnacksBoard : etherSnacksBoards)
         etherSnacksBoard.readSensors();
      for (YoSensorInterface yoEtherSnacksSensor : yoEtherSnacksSensors)
         yoEtherSnacksSensor.update();
      etherSnacksReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - etherSnacksReadStartTime);

      // Read all actuators/mechanisms
      long mechanismReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      totalMeasuredMotorCurrent.set(0.0);
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.read(measuredJointData);

         areMotorsFaulted.set(areMotorsFaulted.getBooleanValue() || mechanismManager.isMotorFaulted());

         //Checking if any motors have over-heated on robot side
         totalMeasuredMotorCurrent.add(mechanismManager.getTotalMeasuredMotorCurrent());
         if (mechanismManager.getIsStatorAboveRecommendedTemperature())
         {
            isMotorWarm.set(true);
         }

         if (mechanismManager.getIsStatorAboveShutDownTemperature())
         {
            isMotorWarm.set(true);
            hasMotorOverHeated.set(true);
         }
      }
      mechanismReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - mechanismReadStartTime);

      // Read IMU data
      long imuReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (IMUManagerInterface imuManager : imuManagers)
      {
         imuManager.read(measuredIMUData);
      }
      imuReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - imuReadStartTime);

      // Read the F/T sensors and calibrate them if necessary
      long forceSensorReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      boolean calibrate = false;
      if (calibrateFootForceSensorsAtomic.getAndSet(false))
         calibrate = true;

      for (ForceSensorManagerInterface forceSensorManager : forceSensorManagers)
      {
         if (calibrate)
            forceSensorManager.calibrate();

         forceSensorManager.read(measuredFTData);
      }
      forceSensorReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - forceSensorReadStartTime);

      // Update hardware status manager for hardware status UI
      hardwareStatusManager.updateDeviceStatusHolders();

      readTime.set(RealtimeThread.getCurrentMonotonicClockTime() - readStartTime);
   }

   /**
    * Writes to all writable sensors. In abstract, that includes force sensors, mechanism managers, and ethersnacks boards.
    *
    * @param desiredJointData Map of desired joint data for mechanism managers
    */
   public void write(Map<String, JointDesiredOutputBasics> desiredJointData)
   {
      long writeStartTime = RealtimeThread.getCurrentMonotonicClockTime();

      //write the F/T sensors
      long forceSensorWriteStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (ForceSensorManagerInterface forceSensorManager : forceSensorManagers)
         forceSensorManager.write();
      forceSensorWriteTime.set(RealtimeThread.getCurrentMonotonicClockTime() - forceSensorWriteStartTime);

      //compute actuator desireds from Controller Joint setpoints
      long mechanismWriteStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         if(useMainActuatorBreakFrequencies.getBooleanValue())
         {
            mechanismManager.setPositionBreakFrequency(mainActuatorPositionBreakFrequency.getDoubleValue());
            mechanismManager.setVelocityBreakFrequency(mainActuatorVelocityBreakFrequency.getDoubleValue());
         }
         mechanismManager.setMasterGain(masterGain.getValue());
         mechanismManager.write(desiredJointData); // this also ticks the low level controllers
      }
      mechanismWriteTime.set(RealtimeThread.getCurrentMonotonicClockTime() - mechanismWriteStartTime);

      // Write data to the EtherSnacks boards
      long etherSnacksWriteStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (EtherSnacksBoardInterface etherSnacksBoard : etherSnacksBoards)
         etherSnacksBoard.writeToBoard();
      etherSnacksWriteTime.set(RealtimeThread.getCurrentMonotonicClockTime() - etherSnacksWriteStartTime);

      writeTime.set(RealtimeThread.getCurrentMonotonicClockTime() - writeStartTime);
   }

   /**
    * Shuts down the robot. Use when terminating the program
    */
   public void shutdown()
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
         mechanismManager.shutDown();
      System.out.println("Hardware manager has been shut down");
   }

   /**
    * Report the current state of sensors and any other info to report
    */
   public abstract void doReporting();

   /**
    * Calibrate the robot. Up for interpretation for each robot
    */
   public abstract void calibrateRobot();

   public void addHasRobotOverHeatedListener(RobotOverHeatedListener robotOverHeatedListener)
   {
      this.robotOverHeatedListener = robotOverHeatedListener;
   }

   /**
    * Try to clear faults on each motor
    */
   public void clearMotorFaults()
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.clearFaults();
      }
   }

   /**
    * Set if compensation should be enabled for the actuators
    *
    * @param enableCompensation Enable if true, disable if false
    */
   public void setEnableCompensationEfforts(boolean enableCompensation)
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.setEnableCompensationEfforts(enableCompensation);
      }
   }

   /**
    * Set if the actuators are servoed (Not sure why it is called this, we are just enabling or disabling actuators with this)
    *
    * @param isRobotServoed If true, enable the actuators. If false, disable the actuators
    */
   public void setIsRobotServoed(boolean isRobotServoed)
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.setIsRobotServoed(isRobotServoed);
      }
   }

   /**
    * Set the master gain in the range [0.0, 1.0]. Any other inputs will be clamped to that range
    *
    * @param desiredMasterGain desired master gain for robot
    */
   public void setMasterGain(double desiredMasterGain)
   {
      masterGain.set(MathTools.clamp(desiredMasterGain, 0.0, 1.0));
   }

   /**
    * Returns current master gain
    *
    * @return masterGain current master gain for robot
    */
   public double getCurrentMasterGain()
   {
      return masterGain.getDoubleValue();
   }

   /**
    * @return The yoboolean for seeing if the motors are faulted
    */
   public YoBoolean getAreMotorsFaulted()
   {
      return areMotorsFaulted;
   }

   /**
    * Interface used to define what happens when a robot overheats
    */
   public static interface RobotOverHeatedListener
   {
      void changed(boolean hasRobotOverHeated);
   }
}
