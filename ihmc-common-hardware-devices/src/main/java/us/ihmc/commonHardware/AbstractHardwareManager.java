package us.ihmc.commonHardware;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonHardware.devices.ForceSensorManagerInterface;
import us.ihmc.commonHardware.devices.MechanismManagerInterface;
import us.ihmc.commonHardware.devices.YoSensorInterface;
import us.ihmc.commonHardware.devices.etherCATDevices.h4.etherSnacks.EtherSnacksBoardInterface;
import us.ihmc.commonHardware.devices.genericIMU.IMUManagerInterface;
import us.ihmc.commonHardware.hardwareStatusUI.controllerSide.HardwareStatusManager;
import us.ihmc.commonHardware.mechanisms.YoJointDesiredDataHolder;
import us.ihmc.commons.MathTools;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotics.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
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

   public void shutDown()
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
         mechanismManager.shutDown();
      System.out.println("Hardware manager has been shut down");
   }

   public abstract void doReporting();

   public abstract void calibrateRobot();

   public void setEnableCompensationEfforts(boolean enableCompensation)
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.setEnableCompensationEfforts(enableCompensation);
      }
   }

   public void setIsRobotServoed(boolean isRobotServoed)
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.setIsRobotServoed(isRobotServoed);
      }
   }

   public void clearMotorFaults()
   {
      for (MechanismManagerInterface mechanismManager : mechanismManagers)
      {
         mechanismManager.clearFaults();
      }
   }

   public MechanismManagerInterface[] getMechanismManagers()
   {
      return mechanismManagers;
   }

   public IMUManagerInterface[] getImuManagers()
   {
      return imuManagers;
   }

   public ForceSensorManagerInterface[] getForceSensorManagers()
   {
      return forceSensorManagers;
   }

   public YoBoolean getAreMotorsFaulted()
   {
      return areMotorsFaulted;
   }

   public void setMasterGain(double desiredMasterGain)
   {
      masterGain.set(MathTools.clamp(desiredMasterGain, 0.0, 1.0));
   }
}
