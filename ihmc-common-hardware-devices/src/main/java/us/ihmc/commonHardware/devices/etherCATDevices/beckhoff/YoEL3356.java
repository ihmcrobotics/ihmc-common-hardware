package us.ihmc.commonHardware.devices.etherCATDevices.beckhoff;

import us.ihmc.etherCAT.slaves.beckhoff.*;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * EL3356 Yo Wrapper - (Beckhoff 1 input Differential Analog Input)
 * 
 * @author Owen Winship (adapted from YoEL3356.java)
 */
public class YoEL3356 implements VoltageSensor, YoLoadCellInterface
{
   private String name = getClass().getSimpleName();
   private YoRegistry registry;
   private final EL3356 el3356;

   private final YoDouble yoValue;
   private final YoBoolean yoEnableHighSpeedMode;
   private final YoBoolean yoEnableCalibration;
   private final YoBoolean yoStartCalibration;
   private final YoBoolean yoStopCalibration;
   private final YoBoolean yoFreezeInput;
   private final YoBoolean yoResumeInput;

   private final YoBoolean yoIsOverRange;
   private final YoBoolean yoIsDataInvalid;
   private final YoBoolean yoError;
   private final YoBoolean yoIsCalibrationInProgress;
   private final YoBoolean yoIsSteadyState;
   private final YoBoolean yoIsSyncError;

   public YoEL3356(EL3356 el3356, String prefix, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix);
      this.el3356 = el3356;
      parentRegistry.addChild(registry);

      yoValue = new YoDouble(prefix + "value", registry);
      //toggle buttons
      yoEnableHighSpeedMode = new YoBoolean(prefix + "EnableHighSpeedMode", registry);
      yoEnableHighSpeedMode.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (v.isZero())
               el3356.disableHighSpeedMode();
            else
               el3356.enableHighSpeedMode();

         }
      });
      yoEnableCalibration = new YoBoolean(prefix + "EnableCalibration", registry);
      yoEnableCalibration.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (v.isZero())
               el3356.disableCalibration();
            else
               el3356.enableCalibration();
         }
      });
      yoStartCalibration = new YoBoolean(prefix + "StartCalibration", registry);
      yoStartCalibration.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            el3356.startCalibration();
            v.setValueFromDouble(0, false);
         }
      });
      yoStopCalibration = new YoBoolean(prefix + "StopCalibration", registry);
      yoStopCalibration.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            el3356.stopCalibration();
            v.setValueFromDouble(0, false);
         }
      });
      yoFreezeInput = new YoBoolean(prefix + "FreezeInput", registry);
      yoFreezeInput.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            el3356.freezeInput();
            v.setValueFromDouble(0, false);
         }
      });
      yoResumeInput = new YoBoolean(prefix + "ResumeInput", registry);
      yoResumeInput.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            el3356.resumeInput();
            v.setValueFromDouble(0, false);
         }
      });

      //diagnostic methods
      yoIsOverRange = new YoBoolean(prefix + "IsOverRange", registry);
      yoIsDataInvalid = new YoBoolean(prefix + "IsDataInvalid", registry);
      yoError = new YoBoolean(prefix + "error", registry);
      yoIsCalibrationInProgress = new YoBoolean(prefix + "IsCalibrationInProgress", registry);
      yoIsSteadyState = new YoBoolean(prefix + "IsSteadyState", registry);
      yoIsSyncError = new YoBoolean(prefix + "IsSyncError", registry);

   }

   public double getVoltageForChannel(int channel) //needed for using YoAnalogSignalWrapper
   {
      return el3356.value();
   }

   @Override
   public double getForce()
   {
      return el3356.value();
   }
   
   @Override
   public YoDouble getYoForce()
   {
	   return yoValue;
   }
   
   @Override
   public void read()
   {
      yoIsOverRange.set(el3356.isOverRange());
      yoIsDataInvalid.set(el3356.isDataInvalid());
      yoError.set(el3356.isError());
      yoIsCalibrationInProgress.set(el3356.isCalibrationInProgress());
      yoIsSteadyState.set(el3356.isSteadyState());
      yoIsSyncError.set(el3356.isSyncError());


      yoValue.set(el3356.value());
   }

}
