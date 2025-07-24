package us.ihmc.commonHardware.devices.etherCATDevices.elmo;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ElmoTwitterStatusRegisterEnums
{

   /**
    * This enum holds the operational statuses of the twitter
    */
   public enum AmplifierStatus
   {
      ALL_OK, UNDERVOLTAGE, OVERVOLTAGE, SAFETY, SHORT_PROTECTION, OVER_TEMPERATURE, ADDITIONAL_ABORT
   }

   /**
    * This enum indicates the possible reference modes
    */
   public enum ReferenceMode
   {
      EXTERNAL_REFERENCE_GENERATOR_DISABLED, EXTERNAL_REFERENCE_GENERATOR_ENABLED
   }

   /**
    * List of possible comtrol modes available on the twitter
    */
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

   /**
    * List of operational statuses of the recorder on the twitter
    */
   public enum RecorderStatus
   {
      NOT_ACTIVE, WAITING_FOR_TRIGGER, COMPLETED, RECORDING
   }

   private ElmoTwitterStatusRegisterEnums()
   {
   }
}
