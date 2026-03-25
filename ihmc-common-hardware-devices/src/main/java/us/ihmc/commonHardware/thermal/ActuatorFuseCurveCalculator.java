package us.ihmc.commonHardware.thermal;

import java.util.ArrayList;
import java.util.List;

/**
 * Computes the fuse curve (maximum peak-current duration vs. current) for an actuator modeled
 * as a two-node thermal network (winding + housing).
 *
 * <p>The model assumes the robot starts thermally soaked at ambient temperature. For a given
 * constant current, it computes the time until the winding temperature reaches
 * {@link ActuatorThermalParameters#getMaxWindingTemperature()}.
 *
 * <p>Winding resistance is temperature-dependent via the copper coefficient:
 * {@code R(T) = R_0 * (1 + ALPHA_CU * (T - T_0))}
 *
 * <p>This is a Java translation of {@code fusePlots2.m}.
 */
public class ActuatorFuseCurveCalculator
{
   /** Temperature coefficient of copper resistance [1/°C]. */
   public static final double ALPHA_CU = 0.00393;

   private static final int BISECTION_ITERATIONS = 100;
   private static final double BISECTION_UPPER_BOUND_SECONDS = 1.0e7;

   private final ActuatorThermalParameters parameters;

   // Pre-computed current-independent terms from the linearized system matrix.
   private final double a12; // = 1 / (C_w * R_wh)
   private final double a21; // = 1 / (C_h * R_wh)
   private final double a22; // = -(R_ha + R_wh) / (C_h * R_wh * R_ha)

   public ActuatorFuseCurveCalculator(ActuatorThermalParameters parameters)
   {
      this.parameters = parameters;

      double Cw  = parameters.getWindingThermalCapacitance();
      double Ch  = parameters.getHousingThermalCapacitance();
      double Rwh = parameters.getWindingToHousingThermalResistance();
      double Rha = parameters.getHousingToAmbientThermalResistance();

      a12 = 1.0 / (Cw * Rwh);
      a21 = 1.0 / (Ch * Rwh);
      a22 = -(Rha + Rwh) / (Ch * Rwh * Rha);
   }

   /**
    * Maximum RMS current [A] that can be sustained indefinitely at the given ambient temperature
    * without exceeding {@link ActuatorThermalParameters#getMaxWindingTemperature()}.
    */
   public double computeSteadyStateMaxCurrent(double ambientTemp)
   {
      double R0   = parameters.getWindingResistanceAtReferenceTemperature();
      double T0   = parameters.getReferenceTemperature();
      double Tmax = parameters.getMaxWindingTemperature();
      double Rha  = parameters.getHousingToAmbientThermalResistance();
      double Rwh  = parameters.getWindingToHousingThermalResistance();

      return Math.sqrt((Tmax - ambientTemp) / (R0 * (Rha + Rwh) * (1.0 + ALPHA_CU * (Tmax - T0))));
   }

   /**
    * Computes the maximum duration [s] that a constant {@code current} [A] can be applied before
    * the winding temperature reaches {@link ActuatorThermalParameters#getMaxWindingTemperature()},
    * assuming both nodes start at {@code ambientTemp}.
    *
    * @param current     applied current [A]; must exceed the steady-state maximum for the result
    *                    to be finite
    * @param ambientTemp ambient (initial) temperature [°C]
    * @return time [s] at which winding temperature reaches T_max
    */
   public double computeMaxPeakDuration(double current, double ambientTemp)
   {
      double R0   = parameters.getWindingResistanceAtReferenceTemperature();
      double T0   = parameters.getReferenceTemperature();
      double Tmax = parameters.getMaxWindingTemperature();
      double Rha  = parameters.getHousingToAmbientThermalResistance();
      double Rwh  = parameters.getWindingToHousingThermalResistance();
      double Ch   = parameters.getHousingThermalCapacitance();
      double Cw   = parameters.getWindingThermalCapacitance();
      double ic2  = current * current;

      // Steady-state winding and housing temperatures under constant current.
      // Derived by setting dT_w/dt = dT_h/dt = 0 with temperature-dependent resistance.
      double denominator = ic2 * R0 * ALPHA_CU * (Rha + Rwh) - 1.0;
      double Tssw = (ic2 * R0 * (ALPHA_CU * T0 - 1.0) * (Rha + Rwh) - ambientTemp) / denominator;
      double Tssh = (ic2 * R0 * (Rha * ALPHA_CU * T0 + Rwh * ALPHA_CU * ambientTemp - Rha) - ambientTemp) / denominator;

      // Eigenvalues of the linearized 2×2 system matrix (current-dependent due to resistive heating).
      double A = Ch * Rha * (ic2 * R0 * Rwh * ALPHA_CU - 1.0);
      double B = Cw * (Rha + Rwh);
      double lambdaA = (A - B) / (2.0 * Ch * Cw * Rwh * Rha);
      double lambdaB = Math.sqrt((A + B) * (A + B) + 4.0 * Ch * Cw * Rha * Rha)
                       / (2.0 * Ch * Cw * Rwh * Rha);
      double lambda1 = lambdaA + lambdaB;
      double lambda2 = lambdaA - lambdaB;

      // Coefficients of the homogeneous solution, set by initial conditions T_w(0) = T_h(0) = ambientTemp.
      double dLambda = lambda1 - lambda2;
      double K1 = (lambda1 - a22) * (a21 * (ambientTemp - Tssw) - (lambda2 - a22) * (ambientTemp - Tssh))
                  / (a21 * dLambda);
      double K2 = (lambda2 - a22) * ((lambda1 - a22) * (ambientTemp - Tssh) - a21 * (ambientTemp - Tssw))
                  / (a21 * dLambda);

      // Winding temperature as a function of time:
      //   T_w(t) = K1*exp(lambda1*t) + K2*exp(lambda2*t) + Tssw
      // Find t such that T_w(t) = Tmax.
      // At t=0: T_w = ambientTemp < Tmax  →  f(0) < 0
      // For current > i_ssmax: Tssw > Tmax, so temperature rises past Tmax at finite t  →  f(hi) > 0
      return bisect(lambda1, lambda2, K1, K2, Tssw, Tmax);
   }

   /**
    * Generates the full fuse curve for the given ambient temperature.
    *
    * <p>Iterates over integer currents starting from the steady-state maximum (rounded up) and
    * continuing until the time-to-failure drops below {@code minPeakDuration}.
    *
    * @param ambientTemp      ambient temperature [°C]
    * @param minPeakDuration  minimum duration threshold [s]; iteration stops when t falls below this
    * @return 2D array where each row is {@code [current [A], maxPeakDuration [s]]}
    */
   public double[][] computeFuseCurve(double ambientTemp, double minPeakDuration)
   {
      int iStart = (int) Math.ceil(computeSteadyStateMaxCurrent(ambientTemp));
      List<double[]> fuseCurve = new ArrayList<>();

      for (int ic = iStart; ; ic++)
      {
         double t = computeMaxPeakDuration(ic, ambientTemp);
         if (t < minPeakDuration)
            break;
         fuseCurve.add(new double[] {ic, t});
      }

      return fuseCurve.toArray(new double[0][]);
   }

   /**
    * Bisection root-find for the time t at which the winding temperature equals T_max.
    *
    * <p>f(t) = K1*exp(lambda1*t) + K2*exp(lambda2*t) + Tssw - Tmax
    * <p>Precondition: f(0) < 0 (temperature starts below T_max).
    */
   private static double bisect(double lambda1, double lambda2, double K1, double K2, double Tssw, double Tmax)
   {
      double lo = 0.0;
      double hi = BISECTION_UPPER_BOUND_SECONDS;

      for (int iter = 0; iter < BISECTION_ITERATIONS; iter++)
      {
         double mid = 0.5 * (lo + hi);
         if (evaluate(lambda1, lambda2, K1, K2, Tssw, Tmax, mid) < 0.0)
            lo = mid;
         else
            hi = mid;
      }

      return 0.5 * (lo + hi);
   }

   private static double evaluate(double lambda1, double lambda2, double K1, double K2,
                                   double Tssw, double Tmax, double t)
   {
      // Guard against overflow: exp() overflows to Infinity above ~709; treat as large positive.
      double exp1 = lambda1 * t > 709.0 ? Double.MAX_VALUE : Math.exp(lambda1 * t);
      double exp2 = lambda2 * t < -709.0 ? 0.0 : Math.exp(lambda2 * t);
      return K1 * exp1 + K2 * exp2 + Tssw - Tmax;
   }
}
