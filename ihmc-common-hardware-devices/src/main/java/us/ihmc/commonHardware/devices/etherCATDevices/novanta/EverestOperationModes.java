package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

import us.ihmc.log.LogTools;

public enum EverestOperationModes
{
   VOLTAGE((byte) -1),
   CURRENT_AMPLIFIER((byte) -4),
   CURRENT((byte) -2),
   PROFILE_VELOCITY((byte) 3),
   PROFILE_POSITION((byte) 1),
   HOMING((byte) 6),
   CYCLIC_SYNCHRONOUS_CURRENT((byte) -3),
   CYCLIC_SYNCHRONOUS_POSITION((byte) 8),
   CYCLIC_SYNCHRONOUS_VELOCITY((byte) 9),
   CYCLIC_SYNCHRONOUS_TORQUE((byte) 10);

   private final byte value;
   public static final EverestOperationModes[] values = values();

   EverestOperationModes(byte value)
   {
      this.value = value;
   }

   public byte getValue()
   {
      return value;
   }

   public static EverestOperationModes fromByte(byte value)
   {
      for (EverestOperationModes operationMode : values)
      {
         if (operationMode.getValue() == value)
            return operationMode;
      }
      LogTools.warn("Gave an incorrect operation mode value " + value + ", reseting to default, CURRENT");
      return CURRENT;
   }

}
