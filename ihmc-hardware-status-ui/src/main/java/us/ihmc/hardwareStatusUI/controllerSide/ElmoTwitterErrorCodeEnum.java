package us.ihmc.hardwareStatusUI.controllerSide;

public enum ElmoTwitterErrorCodeEnum
{
   FEEDBACK_ERROR(0x7300),
   COMMUTATION_FAIL(0x7382),
   HALL_FEEDBACK_MISMATCH(0x7380),
   PEAK_CURRENT_EXCEEDED(0x8311),
   EXTERNAL_INHIBIT(0x5441),
   PHASE_LOSS(0x3130),
   HALL_SPEED_TOO_HIGH(0x7381),
   SPEED_TRACKING_ERROR(0x8480),
   POSITION_TRACKING_ERROR(0x8611),
   GANTRY_YAW_ERROR_EXCEEDED(0x5280),
   COMMUNICATION_FAIL(0x8130),
   UNDER_VOLTAGE(0x3120),
   OVER_VOLTAGE(0x3310),
   STO_DISABLED(0xFF20),
   SHORT_PROTECTION(0x2340),
   MOTOR_OVER_TEMPERATURE(0x4210),
   OVER_SPEED(0x8481),
   SLT_FAULT(0x6180),
   VECTOR_AXIS_FAULT(0x6181),
   MOTOR_STUCK(0x7121),
   FEEDBACK_POSITION_OUT_OF_LIMITS(0x8680),
   KINEMATICS_ERROR(0xFF34),
   GANTRY_SLAVE_DISABLED(0xFF40),
   ADDITIONAL_ABORT_ACTIVE(0x5442),
   DRIVE_OVER_TEMPERATURE(0x4310),
   GANTRY_MASTER_FAULT(0xFF35),
   ATTACHED_DRIVE_FAULT(0xFF50),
   ENABLE_FAILED(0xFF10),
   LOCAL_DISABLE(0xFF30),
   NO_ERROR(0x0);

   private final int errorCode;

   public static ElmoTwitterErrorCodeEnum[] values = ElmoTwitterErrorCodeEnum.values();

   ElmoTwitterErrorCodeEnum(int errorCode)
   {
      this.errorCode = errorCode;
   }

   public int getErrorCode()
   {
      return errorCode;
   }

   public static ElmoTwitterErrorCodeEnum decode(int errorCode)
   {
      for (ElmoTwitterErrorCodeEnum error : values)
      {
         if (error.getErrorCode() == errorCode)
            return error;
      }
      return null;
   }


}
