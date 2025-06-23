package us.ihmc.hardwareStatusUI.controllerSide;

import us.ihmc.etherCAT.master.Slave;
import us.ihmc.etherCAT.master.Slave.State;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class DeviceStatusHolder
{
   public static final String IS_RESPONDING_SUFFIX = "_isResponding";
   public static final String READ_STATUS_SUFFIX = "_readStatus";
   public static final String WRITE_STATUS_SUFFIX = "_writeStatus";
   public static final String STATE_SUFFIX = "_state";

   private final YoRegistry registry;

   private final String name;

   private final DeviceStatusProvider deviceStatusProvider;

   private final YoBoolean isResponding;
   private final OptionalFactoryField<YoEnum<Slave.State>> state;
   private final int primaryAddress;
   private final int secondaryAddress;

   /**
    * Class responsible for holding and YoVariableizing device status info for a given device.
    * Each {@code DeviceStatusHolder} populates its data from its corresponding {@code DeviceStatusProvider}.
    * This class ensure that the appropriate data is registered in the YoRegistry with the correct YoVariable
    * name such that the {@code AbstractUIHardwareStatusManager} can reference this data using those same
    * variable names via a standardized naming format.
    */
   public DeviceStatusHolder(String name, DeviceStatusProvider deviceStatusProvider, YoRegistry registry)
   {
      this(name, -1, -1, deviceStatusProvider, registry);
   }

   /**
    * Initializes device status holder for device with addresses
    * @param name Name of the device
    * @param primaryAddress primary address of the device. For EtherCAT, it is the alias, and for CAN, it is the line number
    * @param secondaryAddress secondary address of the device. For EtherCAT, it is the position, and for CAN, it is the ID number
    * @param deviceStatusProvider status provider for the device
    * @param registry registry of the device status holder
    */
   public DeviceStatusHolder(String name, int primaryAddress, int secondaryAddress, DeviceStatusProvider deviceStatusProvider, YoRegistry registry)
   {
      this.name = name;
      this.deviceStatusProvider = deviceStatusProvider;
      this.registry = registry;
      this.primaryAddress = primaryAddress;
      this.secondaryAddress = secondaryAddress;


      isResponding = new YoBoolean(name + IS_RESPONDING_SUFFIX, registry);
      state = new OptionalFactoryField<>(name + STATE_SUFFIX);

      update();
   }

   public void update()
   {
      setIsResponding(deviceStatusProvider.isResponding());

      if (deviceStatusProvider instanceof EtherCATDeviceStatusProvider etherCATDeviceStatusProvider)
      {
         setState(etherCATDeviceStatusProvider.getState());
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

   public void addStateDataHolder()
   {
      if (!this.state.hasValue())
         state.set(new YoEnum<>(name + STATE_SUFFIX, registry, Slave.State.class));
   }

   public boolean isResponding()
   {
      return deviceStatusProvider.isResponding();
   }
   public Slave.State getState()
   {
      return this.state.hasValue() ? state.get().getValue() : null;
   }

   public int getPrimaryAddress()
   {
      return primaryAddress;
   }

   public int getSecondaryAddress()
   {
      return secondaryAddress;
   }

   public String getName()
   {
      return name;
   }
}
