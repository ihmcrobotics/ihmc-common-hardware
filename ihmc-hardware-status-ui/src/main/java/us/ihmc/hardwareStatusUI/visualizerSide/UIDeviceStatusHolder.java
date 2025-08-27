package us.ihmc.hardwareStatusUI.visualizerSide;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.value.ChangeListener;
import us.ihmc.hardwareStatusUI.visualizerSide.AbstractUIHardwareStatusManager.DeviceType;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

/**
 * Holds all the status information for a specific EtherCAT device, including identifying information, list of child devices, and statuses
 */
public class UIDeviceStatusHolder
{
   protected final SimpleStringProperty name = new SimpleStringProperty("");
   protected final SimpleStringProperty childDescription = new SimpleStringProperty("");
   protected final SimpleStringProperty description = new SimpleStringProperty("");
   protected final SimpleBooleanProperty isResponding = new SimpleBooleanProperty(false);

   protected final SimpleStringProperty readStatus = new SimpleStringProperty("");
   protected final SimpleStringProperty writeStatus = new SimpleStringProperty("");
   protected final SimpleStringProperty state = new SimpleStringProperty("");

   protected final SimpleStringProperty id = new SimpleStringProperty("");

   protected final SimpleBooleanProperty isFaulted = new SimpleBooleanProperty(false);
   protected final SimpleStringProperty underVoltage = new SimpleStringProperty("");
   protected final SimpleStringProperty overVoltage = new SimpleStringProperty("");
   protected final SimpleStringProperty stoDisabled = new SimpleStringProperty("");
   protected final SimpleStringProperty currentShort = new SimpleStringProperty("");
   protected final SimpleStringProperty overTemp = new SimpleStringProperty("");
   protected final SimpleStringProperty elmoErrorCode = new SimpleStringProperty("");
   protected final SimpleStringProperty inputEncoderError = new SimpleStringProperty("");
   protected final SimpleStringProperty outputEncoderError = new SimpleStringProperty("");

   protected DeviceType deviceType;

   protected final ArrayList<UIDeviceStatusHolder> childDevices = new ArrayList<>();

   /**
    * Create a UI device status holder for an EtherCAT device
    *
    * @param name             Name of the device
    * @param description      Description of the device
    * @param childDescription Description of the child device
    * @param isResponding     {@code YoBoolean} to set and track if the device is responding
    * @param state            Tracker of the state of the EtherCAT device
    * @param position         EtherCAT position
    * @param alias            EtherCAT alias
    * @param deviceType       Type of device
    */
   public UIDeviceStatusHolder(String name,
                               String description,
                               String childDescription,
                               YoBoolean isResponding,
                               YoEnumAsStringProperty<Slave.State> state,
                               int position,
                               int alias,
                               DeviceType deviceType)
   {
      initializeCommonProperties(name, description, childDescription, isResponding, deviceType);

      this.id.set("EtherCAT " + alias + "-" + position);
      this.state.set(state.getValue());

      state.addListener(change -> this.state.set(state.getValue()));
   }

   public UIDeviceStatusHolder(String name,
                               String description,
                               String childDescription,
                               YoBoolean isResponding,
                               YoEnumAsStringProperty<Slave.State> state,
                               int position,
                               int alias,
                               DeviceType deviceType,
                               YoBoolean isFaulted,
                               YoBoolean underVoltage,
                               YoBoolean overVoltage,
                               YoBoolean stoDisabled,
                               YoBoolean currentShort,
                               YoBoolean overTemp,
                               YoInteger elmoErrorCode,
                               YoDouble inputEncoderError,
                               YoDouble outputEncoderError)
   {
      this(name, description, childDescription, isResponding, state, position, alias, deviceType);

      this.isFaulted.set(isFaulted.getValue());
      this.underVoltage.set(underVoltage.toString());
      this.overVoltage.set(overVoltage.toString());
      this.stoDisabled.set(stoDisabled.toString());
      this.currentShort.set(currentShort.toString());
      this.overTemp.set(overTemp.toString());
      this.elmoErrorCode.set(elmoErrorCode.toString());
      this.inputEncoderError.set(inputEncoderError.toString());
      this.outputEncoderError.set(outputEncoderError.toString());

      isFaulted.addListener(change -> this.isFaulted.set(isFaulted.getValue()));
      underVoltage.addListener(change -> this.underVoltage.set(underVoltage.toString()));
      overVoltage.addListener(change -> this.overVoltage.set(overVoltage.toString()));
      stoDisabled.addListener(change -> this.stoDisabled.set(stoDisabled.toString()));
      currentShort.addListener(change -> this.currentShort.set(currentShort.toString()));
      overTemp.addListener(change -> this.overTemp.set(overTemp.toString()));
      elmoErrorCode.addListener(change -> this.elmoErrorCode.set(elmoErrorCode.toString()));
      inputEncoderError.addListener(change -> this.inputEncoderError.set(inputEncoderError.toString()));
      outputEncoderError.addListener(change -> this.outputEncoderError.set(outputEncoderError.toString()));
   }

   /**
    * Create a device status holder for a CAN device
    *
    * @param name             Name of the device
    * @param description      Description of the device
    * @param childDescription Description of the child device
    * @param isResponding     {@code YoBoolean} to set and track if the device is responding
    * @param readStatus       Initial CAN read status of the device
    * @param writeStatus      Initial CAN write status of the device
    * @param canID            ID of the device
    * @param deviceType       Type of device
    */
   public UIDeviceStatusHolder(String name,
                               String description,
                               String childDescription,
                               YoBoolean isResponding,
                               String readStatus,
                               String writeStatus,
                               String canID,
                               DeviceType deviceType)
   {
      initializeCommonProperties(name, description, childDescription, isResponding, deviceType);

      this.readStatus.set(readStatus);
      this.writeStatus.set(writeStatus);
      this.id.set("CAN " + canID);

      this.deviceType = deviceType;
   }

   /**
    * Initialize any common properties between types of devices
    *
    * @param name             Name of the device
    * @param description      Description of the device
    * @param childDescription Description of a child of the device
    * @param isResponding     {@code YoBoolean} tracking if the device is responding
    * @param deviceType       Type of device being initialized
    */
   private void initializeCommonProperties(String name, String description, String childDescription, YoBoolean isResponding, DeviceType deviceType)
   {
      this.name.set(name);
      this.description.set(description);
      this.childDescription.set(childDescription);
      this.isResponding.set(isResponding.getBooleanValue());
      this.deviceType = deviceType;

      isResponding.addListener(change -> this.isResponding.set(isResponding.getBooleanValue()));
   }

   /**
    * @param listener Listener to track if device is responding
    */
   public void addDataBooleanChangeListener(ChangeListener<? super Boolean> listener)
   {
      isResponding.addListener(listener);
      isFaulted.addListener(listener);
   }

   /**
    * @param listener Listener to track any changes in read status, write status, or state
    */
   public void addDataStringChangeListener(ChangeListener<? super String> listener)
   {
      readStatus.addListener(listener);
      writeStatus.addListener(listener);
      state.addListener(listener);
      underVoltage.addListener(listener);
      overVoltage.addListener(listener);
      stoDisabled.addListener(listener);
      currentShort.addListener(listener);
      overTemp.addListener(listener);
      elmoErrorCode.addListener(listener);
      inputEncoderError.addListener(listener);
      outputEncoderError.addListener(listener);
   }

   /**
    * Add a child device to the list of child devices
    *
    * @param childDevice Child device to be added
    */
   public void addChildDevice(UIDeviceStatusHolder childDevice)
   {
      this.childDevices.add(childDevice);
   }

   public boolean hasChildDevices()
   {
      return childDevices.size() > 0;
   }

   public String getName()
   {
      return name.get();
   }

   public String getChildDescription()
   {
      return childDescription.get();
   }

   public String getDescription()
   {
      return description.get();
   }

   public Boolean getIsResponding()
   {
      return isResponding.get();
   }

   public String getReadStatus()
   {
      return readStatus.get();
   }

   public String getWriteStatus()
   {
      return writeStatus.get();
   }

   public String getState()
   {
      return state.get();
   }

   public String getId()
   {
      return id.get();
   }

   public DeviceType getDeviceType()
   {
      return deviceType;
   }
}
