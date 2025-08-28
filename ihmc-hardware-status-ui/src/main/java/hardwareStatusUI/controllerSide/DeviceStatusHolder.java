package hardwareStatusUI.controllerSide;

import org.jline.utils.Log;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.log.LogTools;
import us.ihmc.tools.factories.OptionalFactoryField;
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
   private final OptionalFactoryField<YoEnum<Slave.State>> state;

   private final OptionalFactoryField<YoBoolean> isFaulted;
   private final OptionalFactoryField<YoBoolean> underVoltage;
   private final OptionalFactoryField<YoBoolean> overVoltage;
   private final OptionalFactoryField<YoBoolean> stoDisabled;
   private final OptionalFactoryField<YoBoolean> currentShort;
   private final OptionalFactoryField<YoBoolean> overTemp;
   private final OptionalFactoryField<YoInteger> elmoErrorCode;
   private final OptionalFactoryField<YoDouble> inputEncoderError;
   private final OptionalFactoryField<YoDouble> outputEncoderError;

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
      state = new OptionalFactoryField<>(name + STATE_SUFFIX);

      isFaulted = new OptionalFactoryField<>(name + IS_FAULTED_SUFFIX);
      underVoltage = new OptionalFactoryField<>(name + UNDER_VOLTAGE_SUFFIX);
      overVoltage = new OptionalFactoryField<>(name + OVER_VOLTAGE_SUFFIX);
      stoDisabled = new OptionalFactoryField<>(name + STO_DISABLED_SUFFIX);
      currentShort = new OptionalFactoryField<>(name + CURRENT_SHORT_SUFFIX);
      overTemp = new OptionalFactoryField<>(name + OVER_TEMP_SUFFIX);
      elmoErrorCode = new OptionalFactoryField<>(name + ELMO_ERROR_CODE_SUFFIX);
      inputEncoderError = new OptionalFactoryField<>(name + INPUT_ENCODER_ERROR_SUFFIX);
      outputEncoderError = new OptionalFactoryField<>(name + OUTPUT_ENCODER_ERROR_SUFFIX);

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

   public void setIsResponding(boolean isResponding)
   {
      this.isResponding.set(isResponding);
   }

   public void setState(Slave.State state)
   {
      if (!this.state.hasValue())
         addStateDataHolder();

      this.state.get().set(state);
   }

   public void setIsFaulted(boolean isFaulted)
   {
      if (!this.isFaulted.hasValue())
         addIsFaultedDataHolder();

      this.isFaulted.get().set(isFaulted);
   }

   public void setUnderVoltage(boolean underVoltage)
   {
      if (!this.underVoltage.hasValue())
         addUnderVoltageDataHolder();

      this.underVoltage.get().set(underVoltage);
   }

   public void setOverVoltage(boolean overVoltage)
   {
      if (!this.overVoltage.hasValue())
         addOverVoltageDataHolder();

      this.overVoltage.get().set(overVoltage);
   }

   public void setSTODisabled(boolean stoDisabled)
   {
      if (!this.stoDisabled.hasValue())
         addSTODisabledDataHolder();

      this.stoDisabled.get().set(stoDisabled);
   }

   public void setCurrentShort(boolean currentShort)
   {
      if (!this.currentShort.hasValue())
         addCurrentShortDataHolder();

      this.currentShort.get().set(currentShort);
   }

   public void setOverTemp(boolean overTemp)
   {
      if (!this.overTemp.hasValue())
         addOverTempDataHolder();

      this.overTemp.get().set(overTemp);
   }

   public void setElmoErrorCode(int elmoErrorCode)
   {
      if (!this.elmoErrorCode.hasValue())
         addElmoErrorCodeDataHolder();

      this.elmoErrorCode.get().set(elmoErrorCode);
   }

   public void setInputEncoderError(double inputEncoderError)
   {
      if (!this.inputEncoderError.hasValue())
         addInputEncoderErrorDataHolder();

      this.inputEncoderError.get().set(inputEncoderError);
   }

   public void setOutputEncoderError(double outputEncoderError)
   {
      if (!this.outputEncoderError.hasValue())
         addOutputEncoderErrorDataHolder();

      this.outputEncoderError.get().set(outputEncoderError);
   }

   /**
    * Add a state {@code YoEnum} if one hasn't already been created. If there already is one, nothing happens
    */
   public void addStateDataHolder()
   {
      if (!this.state.hasValue())
         state.set(new YoEnum<>(name + STATE_SUFFIX, registry, Slave.State.class));
   }

   public void addIsFaultedDataHolder()
   {
      if (!this.isFaulted.hasValue())
         isFaulted.set(new YoBoolean(name + IS_FAULTED_SUFFIX, registry));
   }

   public void addUnderVoltageDataHolder()
   {
      if (!this.underVoltage.hasValue())
         underVoltage.set(new YoBoolean(name + UNDER_VOLTAGE_SUFFIX, registry));
   }

   public void addOverVoltageDataHolder()
   {
      if (!this.overVoltage.hasValue())
         overVoltage.set(new YoBoolean(name + OVER_VOLTAGE_SUFFIX, registry));
   }

   public void addSTODisabledDataHolder()
   {
      if (!this.stoDisabled.hasValue())
         stoDisabled.set(new YoBoolean(name + STO_DISABLED_SUFFIX, registry));
   }

   public void addCurrentShortDataHolder()
   {
      if (!this.currentShort.hasValue())
         currentShort.set(new YoBoolean(name + CURRENT_SHORT_SUFFIX, registry));
   }

   public void addOverTempDataHolder()
   {
      if (!this.overTemp.hasValue())
         overTemp.set(new YoBoolean(name + OVER_TEMP_SUFFIX, registry));
   }

   public void addElmoErrorCodeDataHolder()
   {
      if (!this.elmoErrorCode.hasValue())
         elmoErrorCode.set(new YoInteger(name + ELMO_ERROR_CODE_SUFFIX, registry));
   }

   public void addInputEncoderErrorDataHolder()
   {
      if (!this.inputEncoderError.hasValue())
         inputEncoderError.set(new YoDouble(name + INPUT_ENCODER_ERROR_SUFFIX, registry));
   }

   public void addOutputEncoderErrorDataHolder()
   {
      if (!this.outputEncoderError.hasValue())
         outputEncoderError.set(new YoDouble(name + OUTPUT_ENCODER_ERROR_SUFFIX, registry));
   }
}
