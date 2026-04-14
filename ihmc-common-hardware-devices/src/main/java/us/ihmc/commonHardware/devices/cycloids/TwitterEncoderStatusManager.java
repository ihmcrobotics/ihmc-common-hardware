package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class TwitterEncoderStatusManager
{
   private enum EncoderState
   {
      NORMAL, WARNING, ERROR
   }

   private final YoDouble encoderWarningValue;
   private final YoDouble encoderErrorValue;

   private final YoEnum<EncoderState> encoderState;
   private EncoderState lastEncoderState = EncoderState.NORMAL;
   private final Timer errorTimer;
   private final YoBoolean encoderPersistentError;
   private final DoubleProvider errorPersistenceThreshold;
   private final String prefix;

   public TwitterEncoderStatusManager(String prefix, DoubleProvider errorPersistenceThreshold, YoRegistry registry)
   {
      this.errorPersistenceThreshold = errorPersistenceThreshold;
      this.prefix = prefix;
      encoderWarningValue = new YoDouble(prefix + "EncoderWarningValue", registry);
      encoderErrorValue = new YoDouble(prefix + "EncoderErrorValue", registry);
      encoderState = new YoEnum<>(prefix + "EncoderState", registry, EncoderState.class);
      encoderPersistentError = new YoBoolean(prefix + "EncoderPersistentError", registry);

      encoderPersistentError.addListener(s ->
                                         {
                                            if (encoderPersistentError.getBooleanValue())
                                            {
                                               LogTools.error(this.prefix + " has a persistent encoder error. You may need to reboot logic");
                                            }
                                         });
      errorTimer = new Timer();
      errorTimer.reset();
   }

   public void update(int warningValue, int errorValue)
   {
      encoderWarningValue.set(warningValue);
      encoderErrorValue.set(errorValue);

      // Updating Encoder State
      if (encoderErrorValue.getDoubleValue() > 0.0)
      {

         if (lastEncoderState != EncoderState.ERROR)
         {
            errorTimer.reset();
            LogTools.warn(prefix + " has experienced an error, position and velocity may be inaccurate");
         }
         else if (errorTimer.isExpired(errorPersistenceThreshold.getValue()))
            encoderPersistentError.set(true);
         encoderState.set(EncoderState.ERROR);
      }
      else if (encoderWarningValue.getDoubleValue() > 0.0)
      {
         encoderState.set(EncoderState.WARNING);
      }
      else
      {
         encoderState.set(EncoderState.NORMAL);
      }
      lastEncoderState = encoderState.getEnumValue();
   }

   public boolean hasPersistentError()
   {
      return encoderPersistentError.getValue();
   }

   public void clearErrors()
   {
      encoderPersistentError.set(false);
      lastEncoderState = EncoderState.NORMAL;
      encoderState.set(EncoderState.NORMAL);
      errorTimer.reset();
   }

}
