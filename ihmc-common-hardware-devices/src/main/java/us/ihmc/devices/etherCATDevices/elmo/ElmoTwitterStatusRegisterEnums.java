package us.ihmc.devices.etherCATDevices.elmo;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ElmoTwitterStatusRegisterEnums
{
   public enum AmplifierStatus
   {
      ALL_OK, UNDERVOLTAGE, OVERVOLTAGE, SAFETY, SHORT_PROTECTION, OVER_TEMPERATURE, ADDITIONAL_ABORT
   }

   public enum ReferenceMode
   {
      EXTERNAL_REFERENCE_GENERATOR_DISABLED, EXTERNAL_REFERENCE_GENERATOR_ENABLED
   }

   public enum ProfileOrMotionMode
   {
      NO_MOTION,
      PROFILE_POSITION_MODE,
      PROFILE_VELOCITY_MODE,
      PROFILE_TORQUE_MODE,
      HOMING_MODE,
      INTERPOLATED_POSITION_MODE,
      CYCLIC_SYNC_POSITION_MODE,
      CYCLIC_SYNC_VELOCITY_MODE,
      CYCLIC_SYNC_TORQUE_MODE,
      N_A
   }

   public enum RecorderStatus
   {
      NOT_ACTIVE, WAITING_FOR_TRIGGER, COMPLETED, RECORDING
   }

   private ElmoTwitterStatusRegisterEnums()
   {
   }
}
