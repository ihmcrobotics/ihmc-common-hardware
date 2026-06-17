package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSILDesiredCurrents
{
   private final YoRegistry registry;

   private YoDouble sil_linearDampingCompensationCurrent;
   private YoDouble sil_coggingCompensationMotorCurrent;
   private YoDouble sil_dahlFrictionCompensationCurrent;
   private YoDouble sil_impedanceControlMotorFeedbackCurrent;
   private YoDouble sil_feedForwardCurrent;
   private YoDouble sil_totalDesiredCurrent;

   public YoSILDesiredCurrents(String prefix, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + getClass().getSimpleName());

      sil_linearDampingCompensationCurrent = new YoDouble(prefix + "_linearDampingCompensationCurrent", registry);
      sil_coggingCompensationMotorCurrent = new YoDouble(prefix + "_coggingCompensationMotorCurrent", registry);
      sil_dahlFrictionCompensationCurrent = new YoDouble(prefix + "_dahlFrictionCompensationCurrent", registry);
      sil_impedanceControlMotorFeedbackCurrent = new YoDouble(prefix + "_impedanceControlMotorFeedbackCurrent", registry);
      sil_feedForwardCurrent = new YoDouble(prefix + "_feedForwardCurrent", registry);
      sil_totalDesiredCurrent = new YoDouble(prefix + "_totalDesiredCurrent", registry);

      parentRegistry.addChild(registry);
   }

   public void update(CycloidPlatinumTwitter platinumTwitter)
   {
      sil_dahlFrictionCompensationCurrent.set(platinumTwitter.getSILDahlFrictionCompensationCurrent());
      sil_linearDampingCompensationCurrent.set(platinumTwitter.getSILLinearDampingCompensationCurrent());
      sil_coggingCompensationMotorCurrent.set(platinumTwitter.getSILDesiredCoggingCompensationCurrent());
      sil_impedanceControlMotorFeedbackCurrent.set(platinumTwitter.getSILDesiredPDControlFeedbackCurrent());

      sil_feedForwardCurrent.set(platinumTwitter.getSILDesiredFeedForwardCurrent());
      sil_totalDesiredCurrent.set(platinumTwitter.getSILDesiredTotalCurrent());
   }

   public double getFeedbackCurrent()
   {
      return sil_impedanceControlMotorFeedbackCurrent.getDoubleValue();
   }
}
