package us.ihmc.commonHardware.thermal;

/**
 * Computes the fuse curve (maximum peak-current duration vs. current) for a specific actuator.
 *
 * <p>This is a convenience wrapper around {@link ThermalModelingTools} that binds
 * {@link ActuatorThermalParameters} at construction time. All computation is delegated to
 * {@link ThermalModelingTools}.
 */
public class ActuatorFuseCurveCalculator
{
   private final ActuatorThermalParameters parameters;

   public ActuatorFuseCurveCalculator(ActuatorThermalParameters parameters)
   {
      this.parameters = parameters;
   }

   /**
    * Maximum RMS current [A] that can be sustained indefinitely at the given ambient temperature
    * without exceeding {@link ActuatorThermalParameters#getMaxWindingTemperature()}.
    */
   public double computeSteadyStateMaxCurrent(double ambientTemp)
   {
      return ThermalModelingTools.computeSteadyStateMaxCurrent(parameters, ambientTemp);
   }

   /**
    * Maximum duration [s] a constant {@code current} can be applied before the winding temperature
    * reaches {@link ActuatorThermalParameters#getMaxWindingTemperature()}, assuming both nodes
    * start at ambient temperature.
    *
    * @param current     applied current [A]; must exceed the steady-state maximum
    * @param ambientTemp ambient (initial) temperature [°C]
    */
   public double computeMaxPeakDuration(double current, double ambientTemp)
   {
      return ThermalModelingTools.computeMaxPeakDuration(parameters, current, ambientTemp);
   }

   /**
    * Generates the full fuse curve for the given ambient temperature.
    *
    * @param ambientTemp     ambient temperature [°C]
    * @param minPeakDuration iteration stops when time-to-failure drops below this [s]
    * @return 2D array where each row is {@code [current [A], maxPeakDuration [s]]}
    */
   public double[][] computeFuseCurve(double ambientTemp, double minPeakDuration)
   {
      return ThermalModelingTools.computeFuseCurve(parameters, ambientTemp, minPeakDuration);
   }
}
