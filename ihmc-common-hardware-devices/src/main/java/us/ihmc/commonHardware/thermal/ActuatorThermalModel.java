package us.ihmc.commonHardware.thermal;

/**
 * Stateful two-node thermal model for an actuator.
 *
 * <p>Tracks the winding temperature T_w and housing temperature T_h over time.
 * Each call to {@link #update(double, double)} applies a constant current for a given duration
 * and advances the internal temperatures using the closed-form analytical solution.
 *
 * <p>The governing equations are:
 * <pre>
 *   C_w · dT_w/dt = i² · R_0 · (1 + α·(T_w − T_0))  +  (T_h − T_w) / R_wh
 *   C_h · dT_h/dt =                                      (T_w − T_h) / R_wh  +  (T_a − T_h) / R_ha
 * </pre>
 *
 * <p>The winding resistance's temperature dependence is linearized around the operating point,
 * yielding a 2×2 linear ODE with closed-form solution via matrix exponential.
 */
public class ActuatorThermalModel
{
   private final ActuatorThermalParameters parameters;

   /** Current winding temperature T_w [°C]. */
   private double windingTemperature;
   /** Current housing temperature T_h [°C]. */
   private double housingTemperature;
   /** Ambient (environment) temperature T_a [°C]. */
   private double ambientTemperature;

   /**
    * @param parameters         actuator thermal parameters
    * @param initialTemperature initial temperature of both winding and housing nodes [°C]
    * @param ambientTemperature ambient environment temperature [°C]
    */
   public ActuatorThermalModel(ActuatorThermalParameters parameters,
                               double initialTemperature,
                               double ambientTemperature)
   {
      this.parameters = parameters;
      this.windingTemperature = initialTemperature;
      this.housingTemperature = initialTemperature;
      this.ambientTemperature = ambientTemperature;
   }

   /**
    * Advances the thermal state by applying a constant {@code current} for {@code duration} seconds.
    *
    * <p>The winding and housing temperatures are updated using the analytical matrix exponential
    * solution of the linearized two-node ODE, with the current internal temperatures as initial
    * conditions.
    *
    * @param current  applied current [A]
    * @param duration time interval [s]
    */
   public void update(double current, double duration)
   {
      double newTw = ThermalModelingTools.evaluateWindingTemperature(
            parameters, current, ambientTemperature, windingTemperature, housingTemperature, duration);
      double newTh = ThermalModelingTools.evaluateHousingTemperature(
            parameters, current, ambientTemperature, windingTemperature, housingTemperature, duration);
      windingTemperature = newTw;
      housingTemperature = newTh;
   }

   /**
    * Resets both nodes to the given temperature.
    */
   public void resetTemperature(double temperature)
   {
      windingTemperature = temperature;
      housingTemperature = temperature;
   }

   /** Winding temperature T_w [°C]. */
   public double getWindingTemperature()
   {
      return windingTemperature;
   }

   /** Housing temperature T_h [°C]. */
   public double getHousingTemperature()
   {
      return housingTemperature;
   }

   /** Ambient temperature T_a [°C]. */
   public double getAmbientTemperature()
   {
      return ambientTemperature;
   }

   /**
    * Updates the ambient temperature used for housing heat dissipation.
    */
   public void setAmbientTemperature(double ambientTemperature)
   {
      this.ambientTemperature = ambientTemperature;
   }

   /**
    * Whether the winding temperature currently exceeds
    * {@link ActuatorThermalParameters#getMaxWindingTemperature()}.
    */
   public boolean isWindingOverTemperature()
   {
      return windingTemperature >= parameters.getMaxWindingTemperature();
   }
}
