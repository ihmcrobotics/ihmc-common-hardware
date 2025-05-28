package us.ihmc.devices.etherCATDevices.beckhoff;

import us.ihmc.tools.Timer;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoFilteredLoadSensor
{
   private static final double ZERO_BIAS_TIMER_PERIOD = 1.0;

   private final YoRegistry registry;

   private final String name;
   private final YoLoadCellInterface yoLoadcell;
   private final AlphaFilteredYoVariable filteredForce;
   private final YoDouble alphaFilter;
   private final YoDouble alphaFrequency;
   private final YoDouble zeroBias;
   private final YoDouble filteredForceBiasRemoved;
   private final YoBoolean zeroLoadCell;
   private final Timer zeroBiasTimer;
   private double zeroBiasSum;
   private int zeroBiasNumSamples;
   private boolean zeroBiasInit;
   private boolean prevZeroLoadCellValue;

   /**
    * reads and filters a generic YoLoadCell object
    * 
    * @param prefix         prefix for named yovariables in this class
    * @param yoLoadcell     instance of a {@code YoLoadCell}
    * @param controller_dt  period of the controller
    * @param parentRegistry the initial parent registry for this object
    */
   public YoFilteredLoadSensor(String prefix, YoLoadCellInterface yoLoadcell, double controller_dt, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix);
      this.name = prefix;
      this.yoLoadcell = yoLoadcell;
      this.alphaFilter = new YoDouble(prefix + "_alphaFilter", registry);
      this.alphaFrequency = new YoDouble(prefix + "_alphaFrequency", registry);
      alphaFrequency.set(30.0);
      alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFrequency.getDoubleValue(), controller_dt));
      this.filteredForce = new AlphaFilteredYoVariable(prefix + "filtered", registry, alphaFilter, yoLoadcell.getYoForce());
      this.zeroBias = new YoDouble(prefix + "ZeroBias", registry);
      this.filteredForceBiasRemoved = new YoDouble(prefix + "filteredForceBiasRemoved", registry);
      this.zeroLoadCell = new YoBoolean(prefix + "MeasureZeroBias", registry);
      this.zeroBiasTimer = new Timer();

      this.alphaFrequency.addListener(new YoVariableChangedListener()
      {

         @Override
         public void changed(YoVariable source)
         {
            alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(alphaFrequency.getDoubleValue(), controller_dt));
         }
      });
      
      this.zeroLoadCell.addListener(new YoVariableChangedListener()
      {

         @Override
         public void changed(YoVariable source)
         {
            if (zeroLoadCell.getBooleanValue())
               zeroBiasInit = true;
         }
      });

      parentRegistry.addChild(registry);
   }

   /**
    * reads and filters the load cell output
    */
   public void update()
   {
      yoLoadcell.read();
      filteredForce.update();
      if (zeroLoadCell.getBooleanValue())
      {
         zeroLoadCell.set(measureZeroBias());
      }
      filteredForceBiasRemoved.set(filteredForce.getDoubleValue() - zeroBias.getDoubleValue());
   }

   /**
    * @return filtered force output from the YoLoadCell with zero load offset removed
    */
   public double getFilteredForceBiasRemoved()
   {
      return filteredForceBiasRemoved.getDoubleValue();
   }

   /**
    * @return filtered force output from the YoLoadCell
    */
   public double getFilteredForceWithBias()
   {
      return filteredForce.getDoubleValue();
   }

   public String getName()
   {
      return name;
   }

   public boolean isZeroing()
   {
      return zeroLoadCell.getBooleanValue();
   }

   public void zero()
   {
      zeroLoadCell.set(true, true);
   }

   /**
    * measures the average zero bias over a period of one second. This measured bias will now be
    * removed in the method {@code getFilteredForceBiasRemoved}
    * 
    * @return whether the zero bias is still recording
    */

   public boolean measureZeroBias()
   {
      if (zeroBiasInit)
      {
         zeroBiasNumSamples = 0;
         zeroBiasSum = 0.0;
         zeroBiasTimer.reset();
         zeroBiasInit = false;
      }
      if (zeroBiasTimer.isRunning(ZERO_BIAS_TIMER_PERIOD))
      {
         zeroBiasSum += getFilteredForceWithBias();
         zeroBiasNumSamples += 1;
      }
      else
      {
         zeroBias.set(zeroBiasSum / zeroBiasNumSamples);
      }
      return zeroBiasTimer.isRunning(ZERO_BIAS_TIMER_PERIOD);
   }
}
