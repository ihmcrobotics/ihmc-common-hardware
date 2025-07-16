package us.ihmc.commonHardware.hardwareStatusUI.visualizerSide;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.value.ChangeListener;
import us.ihmc.commonHardware.hardwareStatusUI.visualizerSide.AbstractUIHardwareStatusManager.DeviceType;
import us.ihmc.etherCAT.master.Slave;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;

public class UIDeviceStatusHolder
{
   protected final SimpleStringProperty name = new SimpleStringProperty();
   protected final SimpleStringProperty childDescription = new SimpleStringProperty();
   protected final SimpleStringProperty description = new SimpleStringProperty();
   protected final SimpleBooleanProperty isResponding = new SimpleBooleanProperty();

   protected final SimpleStringProperty readStatus = new SimpleStringProperty();
   protected final SimpleStringProperty writeStatus = new SimpleStringProperty();
   protected final SimpleStringProperty state = new SimpleStringProperty();

   protected final SimpleStringProperty id = new SimpleStringProperty();

   protected final AbstractUIHardwareStatusManager.DeviceType deviceType;

   protected final ArrayList<UIDeviceStatusHolder> childDevices = new ArrayList<>();

   public UIDeviceStatusHolder(String name,
                               String description,
                               String childDescription,
                               YoBoolean isResponding,
                               YoEnumAsStringProperty<Slave.State> state,
                               int position,
                               int alias,
                               AbstractUIHardwareStatusManager.DeviceType deviceType)
   {
      this(name,
           description,
           childDescription,
           isResponding.getBooleanValue(),
           "",
           "",
           state.getValue(),
           "",
           Integer.toString(position),
           Integer.toString(alias),
           deviceType);

      isResponding.addListener(change -> this.isResponding.set(isResponding.getBooleanValue()));
      state.addListener(change -> this.state.set(state.getValue()));
   }

   public UIDeviceStatusHolder(String name,
                               String description,
                               String childDescription,
                               boolean isResponding,
                               String readStatus,
                               String writeStatus,
                               String state,
                               String canID,
                               String position,
                               String alias,
                               AbstractUIHardwareStatusManager.DeviceType deviceType)
   {
      this.name.set(name);
      this.description.set(description);
      this.childDescription.set(childDescription);
      this.isResponding.set(isResponding);

      this.readStatus.set(readStatus);
      this.writeStatus.set(writeStatus);
      this.state.set(state);

      this.id.set("CAN " + canID);

      if (!position.isEmpty() && !alias.isEmpty())
         this.id.set("EtherCAT " + alias + "-" + position);

      this.deviceType = deviceType;
   }

   public void addDataBooleanChangeListener(ChangeListener<? super Boolean> listener)
   {
      isResponding.addListener(listener);
   }

   public void addDataStringChangeListener(ChangeListener<? super String> listener)
   {
      readStatus.addListener(listener);
      writeStatus.addListener(listener);
      state.addListener(listener);
   }

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
