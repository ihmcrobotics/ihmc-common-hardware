package us.ihmc.commonHardware;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonHardware.devices.ForceSensorManagerInterface;
import us.ihmc.commonHardware.devices.MechanismManagerInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksBoardInterface;
import us.ihmc.commonHardware.devices.genericIMU.IMUManagerInterface;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.master.Slave;
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
   protected final YoLong ftReadTime;
   protected final YoLong mechanismReadTime;
   protected final YoLong etherSnacksReadTime;
   protected final YoLong readTime;

   protected final YoLong ftWriteTime;
   protected final YoLong mechanismWriteTime;
   protected final YoLong etherSnacksWriteTime;
   protected final YoLong writeTime;

   protected final YoBoolean areMotorsFaulted;

   protected final YoDouble masterGain;

   protected final HardwareStatusManager hardwareStatusManager;

   protected final YoBoolean isMotorWarm;
   protected final YoBoolean hasMotorOverHeated;
   protected final YoDouble totalMeasuredMotorCurrent;

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
      ftReadTime = new YoLong("ftReadTime", registry);
      mechanismReadTime = new YoLong("mechanismReadTime", registry);
      etherSnacksReadTime = new YoLong("etherSnacksReadTime", registry);
      readTime = new YoLong("readTime", registry);

      ftWriteTime = new YoLong("ftWriteTime", registry);
      mechanismWriteTime = new YoLong("mechanismWriteTime", registry);
      etherSnacksWriteTime = new YoLong("etherSnacksWriteTime", registry);
      writeTime = new YoLong("writeTime", registry);

      hasMotorOverHeated = new YoBoolean("hasArmMotorOverHeated", registry);
      isMotorWarm = new YoBoolean("isArmMotorWarm", registry);
      totalMeasuredMotorCurrent = new YoDouble("totalMeasuredMotorCurrent", registry);

      masterGain = new YoDouble("lowLevelMasterGain", registry);

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

      // Read the ATI F/T sensors and calibrate them if necessary
      long ftReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      boolean calibrate = false;
      if (calibrateFootForceSensorsAtomic.getAndSet(false))
         calibrate = true;

      for (ForceSensorManagerInterface forceSensorManager : forceSensorManagers)
      {
         if (calibrate)
            forceSensorManager.calibrate();

         forceSensorManager.read(measuredFTData);
      }
      ftReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - ftReadStartTime);

      // Read the EtherSnacks boards
      long etherSnacksReadStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (EtherSnacksBoardInterface etherSnacksBoard : etherSnacksBoards)
         etherSnacksBoard.readSensors();
      for (YoSensorInterface yoEtherSnacksSensor : yoEtherSnacksSensors)
         yoEtherSnacksSensor.update();
      etherSnacksReadTime.set(RealtimeThread.getCurrentMonotonicClockTime() - etherSnacksReadStartTime);

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
      long ftWriteStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (ForceSensorManagerInterface forceSensorManager : forceSensorManagers)
         forceSensorManager.write();
      ftWriteTime.set(RealtimeThread.getCurrentMonotonicClockTime() - ftWriteStartTime);

      //compute actuator desireds from Controller Joint setpoints
      long mechanismWriteStartTime = RealtimeThread.getCurrentMonotonicClockTime();
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
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
   public void shutDown()
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
    * @return The yoboolean for seeing if the motors are faulted
    */
   public YoBoolean getAreMotorsFaulted()
   {
      return areMotorsFaulted;
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
}
