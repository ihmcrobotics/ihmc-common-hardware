package us.ihmc.hardwareStatusUI.controllerSide;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class DeviceStatusHolder
{
   public static final String IS_RESPONDING_SUFFIX = "_isResponding";
   public static final String READ_STATUS_SUFFIX = "_readStatus";
   public static final String WRITE_STATUS_SUFFIX = "_writeStatus";
   public static final String STATE_SUFFIX = "_state";

   public static final String IS_FAULTED_SUFFIX = "_isFaulted";
   public static final String UNDER_VOLTAGE_SUFFIX = "_underVoltage";
   public static final String OVER_VOLTAGE_SUFFIX = "_overVoltage";
   public static final String STO_DISABLED_SUFFIX = "_stoDisabled";
   public static final String CURRENT_SHORT_SUFFIX = "_currentShort";
   public static final String OVER_TEMP_SUFFIX = "_overTemp";
   public static final String ELMO_ERROR_CODE_SUFFIX = "_elmoErrorCode";
   public static final String INPUT_ENCODER_ERROR_SUFFIX = "_inputEncoderError";
   public static final String OUTPUT_ENCODER_ERROR_SUFFIX = "_outputEncoderError";

   private final YoRegistry registry;

   private final String name;

   private final DeviceStatusProvider deviceStatusProvider;

   private final YoBoolean isResponding;

   private YoEnum<Slave.State> state = null;
   private YoBoolean isFaulted = null;
   private YoBoolean underVoltage = null;
   private YoBoolean overVoltage = null;
   private YoBoolean stoDisabled = null;
   private YoBoolean currentShort = null;
   private YoBoolean overTemp = null;
   private YoInteger elmoErrorCode = null;
   private YoDouble inputEncoderError = null;
   private YoDouble outputEncoderError = null;

   /**
    * Class responsible for holding and YoVariableizing device status info for a given device.
    * Each {@code DeviceStatusHolder} populates its data from its corresponding {@code DeviceStatusProvider}.
    * This class ensure that the appropriate data is registered in the YoRegistry with the correct YoVariable
    * name such that the {@code AbstractUIHardwareStatusManager} can reference this data using those same
    * variable names via a standardized naming format.
    */
   public DeviceStatusHolder(String name, DeviceStatusProvider deviceStatusProvider, YoRegistry registry)
   {
      this.name = name;
      this.deviceStatusProvider = deviceStatusProvider;
      this.registry = registry;

      isResponding = new YoBoolean(name + IS_RESPONDING_SUFFIX, registry);

      update();
   }

   /**
    * Update the status of the device
    */
   public void update()
   {
      setIsResponding(deviceStatusProvider.isResponding());

      if (deviceStatusProvider instanceof EtherCATDeviceStatusProvider etherCATDeviceStatusProvider)
      {
         setState(etherCATDeviceStatusProvider.getState());
      }

      if (deviceStatusProvider instanceof ElmoTwitterDeviceStatusProvider elmoTwitterDeviceStatusProvider)
      {
         setState(elmoTwitterDeviceStatusProvider.getState());
         setIsFaulted(elmoTwitterDeviceStatusProvider.isFaulted());
         setUnderVoltage(elmoTwitterDeviceStatusProvider.isUnderVoltage());
         setOverVoltage(elmoTwitterDeviceStatusProvider.isOverVoltage());
         setSTODisabled(elmoTwitterDeviceStatusProvider.isSTODisabled());
         setCurrentShort(elmoTwitterDeviceStatusProvider.isCurrentShort());
         setOverTemp(elmoTwitterDeviceStatusProvider.isOverTemp());
         setElmoErrorCode(elmoTwitterDeviceStatusProvider.getElmoErrorCode());
         setInputEncoderError(elmoTwitterDeviceStatusProvider.getInputEncoderError());
         setOutputEncoderError(elmoTwitterDeviceStatusProvider.getOutputEncoderError());
      }
   }

   public String getName()
   {
      return name;
   }

   public DeviceStatusProvider getDeviceStatusProvider()
   {
      return deviceStatusProvider;
   }

   public void setIsResponding(boolean isResponding)
   {
      this.isResponding.set(isResponding);
   }

   public void setState(Slave.State state)
   {
      if (this.state == null)
         addStateDataHolder();

      this.state.set(state);
   }

   public void setIsFaulted(boolean isFaulted)
   {
      if (this.isFaulted == null)
         addIsFaultedDataHolder();

      this.isFaulted.set(isFaulted);
   }

   public void setUnderVoltage(boolean underVoltage)
   {
      if (this.underVoltage == null)
         addUnderVoltageDataHolder();

      this.underVoltage.set(underVoltage);
   }

   public void setOverVoltage(boolean overVoltage)
   {
      if (this.overVoltage == null)
         addOverVoltageDataHolder();

      this.overVoltage.set(overVoltage);
   }

   public void setSTODisabled(boolean stoDisabled)
   {
      if (this.stoDisabled == null)
         addSTODisabledDataHolder();

      this.stoDisabled.set(stoDisabled);
   }

   public void setCurrentShort(boolean currentShort)
   {
      if (this.currentShort == null)
         addCurrentShortDataHolder();

      this.currentShort.set(currentShort);
   }

   public void setOverTemp(boolean overTemp)
   {
      if (this.overTemp == null)
         addOverTempDataHolder();

      this.overTemp.set(overTemp);
   }

   public void setElmoErrorCode(int elmoErrorCode)
   {
      if (this.elmoErrorCode == null)
         addElmoErrorCodeDataHolder();

      this.elmoErrorCode.set(elmoErrorCode);
   }

   public void setInputEncoderError(double inputEncoderError)
   {
      if (this.inputEncoderError == null)
         addInputEncoderErrorDataHolder();

      this.inputEncoderError.set(inputEncoderError);
   }

   public void setOutputEncoderError(double outputEncoderError)
   {
      if (this.outputEncoderError == null)
         addOutputEncoderErrorDataHolder();

      this.outputEncoderError.set(outputEncoderError);
   }

   /**
    * Add a state {@code YoEnum} if one hasn't already been created. If there already is one, nothing happens
    */
   public void addStateDataHolder()
   {
      if (this.state == null)
         state = new YoEnum<>(name + STATE_SUFFIX, registry, Slave.State.class);
   }

   public void addIsFaultedDataHolder()
   {
      if (this.isFaulted == null)
         isFaulted = new YoBoolean(name + IS_FAULTED_SUFFIX, registry);
   }

   public void addUnderVoltageDataHolder()
   {
      if (this.underVoltage == null)
         underVoltage = new YoBoolean(name + UNDER_VOLTAGE_SUFFIX, registry);
   }

   public void addOverVoltageDataHolder()
   {
      if (this.overVoltage == null)
         overVoltage = new YoBoolean(name + OVER_VOLTAGE_SUFFIX, registry);
   }

   public void addSTODisabledDataHolder()
   {
      if (this.stoDisabled == null)
         stoDisabled = new YoBoolean(name + STO_DISABLED_SUFFIX, registry);
   }

   public void addCurrentShortDataHolder()
   {
      if (this.currentShort == null)
         currentShort = new YoBoolean(name + CURRENT_SHORT_SUFFIX, registry);
   }

   public void addOverTempDataHolder()
   {
      if (this.overTemp == null)
         overTemp = new YoBoolean(name + OVER_TEMP_SUFFIX, registry);
   }

   public void addElmoErrorCodeDataHolder()
   {
      if (this.elmoErrorCode == null)
         elmoErrorCode = new YoInteger(name + ELMO_ERROR_CODE_SUFFIX, registry);
   }

   public void addInputEncoderErrorDataHolder()
   {
      if (this.inputEncoderError == null)
         inputEncoderError = new YoDouble(name + INPUT_ENCODER_ERROR_SUFFIX, registry);
   }

   public void addOutputEncoderErrorDataHolder()
   {
      if (this.outputEncoderError == null)
         outputEncoderError = new YoDouble(name + OUTPUT_ENCODER_ERROR_SUFFIX, registry);
   }
}
