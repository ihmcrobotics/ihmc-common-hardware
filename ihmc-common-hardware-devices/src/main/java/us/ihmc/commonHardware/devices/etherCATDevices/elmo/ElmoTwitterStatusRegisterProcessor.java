package us.ihmc.commonHardware.devices.etherCATDevices.elmo;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ElmoTwitterStatusRegisterProcessor
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoEnum<ElmoTwitterStatusRegisterEnums.AmplifierStatus> amplifierStatus;
   private final YoBoolean isServoEnabled;
   private final YoEnum<ElmoTwitterStatusRegisterEnums.ReferenceMode> referenceMode;
   private final YoBoolean faultOccurred;
   private final YoBoolean isElmoHomingOrCaptureActive;
   private final YoEnum<ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode> profileOrMotionMode;
   private final YoBoolean isUserProgramRunning;
   private final YoBoolean isCurrentLimitOn;
   private final YoBoolean isSTO_DSPUnsafe;
   private final YoBoolean isSTO_PWMUnsafe;
   private final YoEnum<ElmoTwitterStatusRegisterEnums.RecorderStatus> recorderStatus;
   private final YoBoolean isTargetReached;
   private final YoBoolean isShuntSwitchedOff;
   private final YoBoolean isMotorOn;
   private final YoBoolean isMoving;
   private final YoBoolean hallAState;
   private final YoBoolean hallBState;
   private final YoBoolean hallCState;
   private final YoBoolean didSTODiagnosticFail;
   private final YoBoolean didProfilerStopDueToSwitch;
   private final YoBoolean isPTPBufferFull;

   /**
    * Creates a yovariable wrapper around all of the information provided by the twitter status register.
    *
    * @param parentRegistry Parent registru of the status register
    */
   public ElmoTwitterStatusRegisterProcessor(YoRegistry parentRegistry)
   {
      String prefix = "SR_";

      amplifierStatus = new YoEnum<>(prefix + "amplifierStatus", registry, ElmoTwitterStatusRegisterEnums.AmplifierStatus.class, false);
      isServoEnabled = new YoBoolean(prefix + "isServoEnabled", registry);
      referenceMode = new YoEnum<>(prefix + "referenceMode", registry, ElmoTwitterStatusRegisterEnums.ReferenceMode.class, false);
      faultOccurred = new YoBoolean(prefix + "faultOccurred", registry);
      isElmoHomingOrCaptureActive = new YoBoolean(prefix + "isElmoHomingOrCaptureActive", registry);
      profileOrMotionMode = new YoEnum<>(prefix + "profileOrMotionMode", registry, ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.class, false);
      isUserProgramRunning = new YoBoolean(prefix + "isUserProgramRunning", registry);
      isCurrentLimitOn = new YoBoolean(prefix + "isCurrentLimitOn", registry);
      isSTO_DSPUnsafe = new YoBoolean(prefix + "isSTO_DSPUnsafe", registry);
      isSTO_PWMUnsafe = new YoBoolean(prefix + "isSTO_PWMUnsafe", registry);
      recorderStatus = new YoEnum<>(prefix + "recorderStatus", registry, ElmoTwitterStatusRegisterEnums.RecorderStatus.class, false);
      isTargetReached = new YoBoolean(prefix + "isTargetReached", registry);
      isShuntSwitchedOff = new YoBoolean(prefix + "isShuntSwitchedOff", registry);
      isMotorOn = new YoBoolean(prefix + "isMotorOn", registry);
      isMoving = new YoBoolean(prefix + "isMoving", registry);
      hallAState = new YoBoolean(prefix + "hallAState", registry);
      hallBState = new YoBoolean(prefix + "hallBState", registry);
      hallCState = new YoBoolean(prefix + "hallCState", registry);
      didSTODiagnosticFail = new YoBoolean(prefix + "didSTODiagnosticFail", registry);
      didProfilerStopDueToSwitch = new YoBoolean(prefix + "didProfilerStopDueToSwitch", registry);
      isPTPBufferFull = new YoBoolean(prefix + "isPTPBufferFull", registry);

      parentRegistry.addChild(this.registry);
   }

   /**
    * Convert the status register integer received from the twitter into the statuses assigned to each bit
    *
    * @param statusRegisterBits Integer representation of the status register
    */
   public void processStatusRegisterBits(int statusRegisterBits)
   {
      // bits 0 -> 3
      processAmplifierStatus(statusRegisterBits);

      // bit 4, servo enabled
      this.isServoEnabled.set(getBitAsBoolean(statusRegisterBits, 4));

      // bit 5
      processReferenceMode(statusRegisterBits);

      // bit 6, has Fault Occured
      this.faultOccurred.set(getBitAsBoolean(statusRegisterBits, 6));

      // bit 7, is Elmo Homing/Capture active
      this.isElmoHomingOrCaptureActive.set(getBitAsBoolean(statusRegisterBits, 7));

      // bits 8 -> 11
      processProfileOrMotionMode(statusRegisterBits);

      // bit 12, is User Program Running
      this.isUserProgramRunning.set(getBitAsBoolean(statusRegisterBits, 12));

      // bit 13, is current limit on
      this.isCurrentLimitOn.set(getBitAsBoolean(statusRegisterBits, 13));

      // bit 14, is STO_DSP unsafe
      this.isSTO_DSPUnsafe.set(getBitAsBoolean(statusRegisterBits, 14));

      // bit 15, is STO_PWM unsafe
      this.isSTO_PWMUnsafe.set(getBitAsBoolean(statusRegisterBits, 15));

      // bits 16 -> 17
      processRecorderStatus(statusRegisterBits);

      // bit 18, is target reached
      this.isTargetReached.set(getBitAsBoolean(statusRegisterBits, 18));

      /* ---- bits 19 and 20 are reserved ---- */

      // bit 21, is shunt switched off
      this.isShuntSwitchedOff.set(getBitAsBoolean(statusRegisterBits, 21));

      // bit 22, is motor on
      this.isMotorOn.set(getBitAsBoolean(statusRegisterBits, 22));

      // bit 23, is moving
      this.isMoving.set(getBitAsBoolean(statusRegisterBits, 23));

      // bits 24, 25, 26 are hall states; meaning of these bits undocumented
      this.hallAState.set(getBitAsBoolean(statusRegisterBits, 24));
      this.hallBState.set(getBitAsBoolean(statusRegisterBits, 25));
      this.hallCState.set(getBitAsBoolean(statusRegisterBits, 26));

      // bit 27, did STO Diagnostics fail
      this.didSTODiagnosticFail.set(getBitAsBoolean(statusRegisterBits, 27));

      // bit 28, did profiler stop due to switch
      this.didProfilerStopDueToSwitch.set(getBitAsBoolean(statusRegisterBits, 28));

      /* ---- bit 29 is reserved ---- */

      // bit 30, is PTP buffer full
      this.isPTPBufferFull.set(getBitAsBoolean(statusRegisterBits, 30));

      /* ---- bit 31 is reserved ---- */
   }

   public boolean hasFaultOccurred()
   {
      return faultOccurred.getBooleanValue();
   }

   /**
    * Convert the recorder status integer received from the twitter into the statuses assigned to each bit
    *
    * @param statusRegisterBits Integer representation of the recorder status
    */
   private void processRecorderStatus(int statusRegisterBits)
   {
      int recorderStatus = getBitRange(statusRegisterBits, 2, 16);
      switch (recorderStatus)
      {
         case 0:
            this.recorderStatus.set(ElmoTwitterStatusRegisterEnums.RecorderStatus.NOT_ACTIVE);
            break;
         case 1:
            this.recorderStatus.set(ElmoTwitterStatusRegisterEnums.RecorderStatus.WAITING_FOR_TRIGGER);
            break;
         case 2:
            this.recorderStatus.set(ElmoTwitterStatusRegisterEnums.RecorderStatus.COMPLETED);
            break;
         case 3:
            this.recorderStatus.set(ElmoTwitterStatusRegisterEnums.RecorderStatus.RECORDING);
            break;
         default:
            break;
      }
   }

   /**
    * Convert the control mode integer received from the twitter into the mode assigned to each bit
    *
    * @param statusRegisterBits Integer representation of the control
    */
   private void processProfileOrMotionMode(int statusRegisterBits)
   {
      int profileOrMotionMode = this.getBitRange(statusRegisterBits, 4, 8);
      switch (profileOrMotionMode)
      {
         case 0:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.NO_MOTION);
            break;
         case 1:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.PROFILE_POSITION_MODE);
            break;
         case 3:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.PROFILE_VELOCITY_MODE);
            break;
         case 4:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.PROFILE_TORQUE_MODE);
            break;
         case 6:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.HOMING_MODE);
            break;
         case 7:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.INTERPOLATED_POSITION_MODE);
            break;
         case 8:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.CYCLIC_SYNC_POSITION_MODE);
            break;
         case 9:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.CYCLIC_SYNC_VELOCITY_MODE);
            break;
         case 10:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.CYCLIC_SYNC_TORQUE_MODE);
            break;
         case 2:
         case 5:
         default:
            this.profileOrMotionMode.set(ElmoTwitterStatusRegisterEnums.ProfileOrMotionMode.N_A);
            break;
      }
   }

   /**
    * Convert the reference mode integer received from the twitter into the reference mode assigned to each bit
    *
    * @param statusRegisterBits Integer representation of the reference mode
    */
   private void processReferenceMode(int statusRegisterBits)
   {
      boolean referenceMode = !getBitAsBoolean(statusRegisterBits, 5);
      this.referenceMode.set(referenceMode ?
                                   ElmoTwitterStatusRegisterEnums.ReferenceMode.EXTERNAL_REFERENCE_GENERATOR_DISABLED :
                                   ElmoTwitterStatusRegisterEnums.ReferenceMode.EXTERNAL_REFERENCE_GENERATOR_ENABLED);
   }

   /**
    * Convert the amplifier status integer received from the twitter into the statuses assigned to each bit
    *
    * @param statusRegisterBits Integer representation of the amplifier status
    */
   private void processAmplifierStatus(int statusRegisterBits)
   {
      int amplifierStatus = getBitRange(statusRegisterBits, 4, 0);

      switch (amplifierStatus)
      {
         case 0:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.ALL_OK);
            break;
         case 3:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.UNDERVOLTAGE);
            break;
         case 5:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.OVERVOLTAGE);
            break;
         case 7:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.SAFETY);
            break;
         case 11:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.SHORT_PROTECTION);
            break;
         case 13:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.OVER_TEMPERATURE);
            break;
         case 15:
            this.amplifierStatus.set(ElmoTwitterStatusRegisterEnums.AmplifierStatus.ADDITIONAL_ABORT);
            break;
         default:
            break;
      }
   }

   /**
    * <p>
    * Extract a range of bits from a larger set of bits, starting at a particular position
    * </p>
    * <p>
    * startingBit starts at position 0 for the least significant bit, and the method will move up from
    * the starting bit.
    * </p>
    * <p>
    * For example: We will look at the bit literal 0b11000101
    * </p>
    * <p>
    * Bits: <br />
    *
    * <pre>
    * | 1 | 1 | 0 | 0 | 0 | 1 | 0 | 1 |
    * </pre>
    *
    * Bit address: | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
    * </p>
    * <p>
    * As such, using this method to do something such as getBitRange(0b11000101, 3, 2) would return
    * 0b001, or just 1.
    * </p>
    *
    * @param rawBits      the bits being processed
    * @param numberOfBits the number of bits to extract
    * @param startingBit  the starting bit
    * @return
    */
   private int getBitRange(int rawBits, int numberOfBits, int startingBit)
   {
      return ((1 << numberOfBits) - 1) & (rawBits >> startingBit);
   }

   private boolean getBitAsBoolean(int rawBits, int bitPosition)
   {
      int result = (rawBits >> bitPosition) & 1;
      return result == 1;
   }
}
