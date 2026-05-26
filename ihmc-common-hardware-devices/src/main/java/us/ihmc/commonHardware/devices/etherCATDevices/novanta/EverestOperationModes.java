package us.ihmc.commonHardware.devices.etherCATDevices.novanta;

public enum EverestOperationModes
{
   VOLTAGE(-1),
   CURRENT_AMPLIFIER(-4),
   CURRENT(-2),
   PROFILE_VELOCITY(3),
   PROFILE_POSITION(1),
   HOMING(6),
   CYCLIC_SYNCHRONOUS_CURRENT(-3),
   CYCLIC_SYNCHRONOUS_POSITION(8),
   CYCLIC_SYNCHRONOUS_VELOCITY(9),
   CYCLIC_SYNCHRONOUS_TORQUE(10);

   private final int value;

   EverestOperationModes(int value)
   {
      this.value = value;
   }

   public int getValue()
   {
      return value;
   }

}
