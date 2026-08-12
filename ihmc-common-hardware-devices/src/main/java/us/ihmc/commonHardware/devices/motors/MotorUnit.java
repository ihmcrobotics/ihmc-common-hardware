package us.ihmc.commonHardware.devices.motors;

public enum MotorUnit
{
   KOLLMORGEN_TBM2G_07626C(new KollMorgenTBM2G07626C()),
   KOLLMORGEN_TBM2G_06826C(new KollMorgenTBM2G06826C()),
   TQILM115x25(new TQILM115x25());

   private final MotorParameters motorParameters;

   MotorUnit(MotorParameters motorParameters)
   {
      this.motorParameters = motorParameters;
   }

   public MotorParameters getMotorParameters()
   {
      return motorParameters;
   }
}
