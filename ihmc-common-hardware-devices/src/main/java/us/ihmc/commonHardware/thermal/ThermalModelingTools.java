package us.ihmc.commonHardware.thermal;

import java.util.ArrayList;
import java.util.List;

/**
 * Static utility methods for the two-node (winding + housing) actuator thermal model.
 *
 * <p>The model differential equations are:
 * <pre>
 *   C_w * dT_w/dt = i² * R_0 * (1 + α*(T_w - T_0))  +  (T_h - T_w) / R_wh
 *   C_h * dT_h/dt =                                      (T_w - T_h) / R_wh  +  (T_a - T_h) / R_ha
 * </pre>
 *
 * <p>The winding resistance is linearized around its temperature-dependent value, yielding a
 * 2×2 linear ODE with a closed-form matrix exponential solution.
 */
public class ThermalModelingTools
{
   /** Temperature coefficient of copper resistance [1/°C]. */
   public static final double ALPHA_CU = 0.00393;

   private static final int    BISECTION_ITERATIONS        = 100;
   private static final double BISECTION_UPPER_BOUND_SECONDS = 1.0e7;

   private ThermalModelingTools()
   {
   }

   // -------------------------------------------------------------------------
   // System matrix constants (current-independent)
   // -------------------------------------------------------------------------

   /** a₁₂ = 1 / (C_w · R_wh) */
   public static double computeA12(MotorThermalParameters p)
   {
      return 1.0 / (p.getWindingThermalCapacitance() * p.getWindingToHousingThermalResistance());
   }

   /** a₂₁ = 1 / (C_h · R_wh) */
   public static double computeA21(MotorThermalParameters p)
   {
      return 1.0 / (p.getHousingThermalCapacitance() * p.getWindingToHousingThermalResistance());
   }

   /** a₂₂ = −(R_ha + R_wh) / (C_h · R_wh · R_ha) */
   public static double computeA22(MotorThermalParameters p)
   {
      double Ch  = p.getHousingThermalCapacitance();
      double Rwh = p.getWindingToHousingThermalResistance();
      double Rha = p.getHousingToAmbientThermalResistance();
      return -(Rha + Rwh) / (Ch * Rwh * Rha);
   }

   // -------------------------------------------------------------------------
   // Steady-state temperatures and eigenvalues (current-dependent)
   // -------------------------------------------------------------------------

   /**
    * Maximum RMS current [A] that can be sustained indefinitely without exceeding
    * {@link MotorThermalParameters#getMaxWindingTemperature()}.
    */
   public static double computeSteadyStateMaxCurrent(MotorThermalParameters p, double ambientTemp)
   {
      double R0   = p.getWindingResistanceAtReferenceTemperature();
      double T0   = p.getReferenceTemperature();
      double Tmax = p.getMaxWindingTemperature();
      double Rha  = p.getHousingToAmbientThermalResistance();
      double Rwh  = p.getWindingToHousingThermalResistance();
      return Math.sqrt((Tmax - ambientTemp) / (R0 * (Rha + Rwh) * (1.0 + ALPHA_CU * (Tmax - T0))));
   }

   /**
    * Steady-state winding temperature [°C] under a constant current and ambient temperature.
    */
   public static double computeSteadyStateWindingTemp(MotorThermalParameters p, double current, double ambientTemp)
   {
      double R0  = p.getWindingResistanceAtReferenceTemperature();
      double T0  = p.getReferenceTemperature();
      double Rha = p.getHousingToAmbientThermalResistance();
      double Rwh = p.getWindingToHousingThermalResistance();
      double ic2 = current * current;
      double denom = ic2 * R0 * ALPHA_CU * (Rha + Rwh) - 1.0;
      return (ic2 * R0 * (ALPHA_CU * T0 - 1.0) * (Rha + Rwh) - ambientTemp) / denom;
   }

   /**
    * Steady-state housing temperature [°C] under a constant current and ambient temperature.
    */
   public static double computeSteadyStateHousingTemp(MotorThermalParameters p, double current, double ambientTemp)
   {
      double R0  = p.getWindingResistanceAtReferenceTemperature();
      double T0  = p.getReferenceTemperature();
      double Rha = p.getHousingToAmbientThermalResistance();
      double Rwh = p.getWindingToHousingThermalResistance();
      double ic2 = current * current;
      double denom = ic2 * R0 * ALPHA_CU * (Rha + Rwh) - 1.0;
      return (ic2 * R0 * (Rha * ALPHA_CU * T0 + Rwh * ALPHA_CU * ambientTemp - Rha) - ambientTemp) / denom;
   }

   /**
    * Larger eigenvalue λ₁ of the linearized system matrix for the given current.
    */
   public static double computeLambda1(MotorThermalParameters p, double current)
   {
      double[] ab = computeLambdaAB(p, current);
      return ab[0] + ab[1];
   }

   /**
    * Smaller eigenvalue λ₂ of the linearized system matrix for the given current.
    */
   public static double computeLambda2(MotorThermalParameters p, double current)
   {
      double[] ab = computeLambdaAB(p, current);
      return ab[0] - ab[1];
   }

   /**
    * Returns [lambdaA, lambdaB] where λ₁ = lambdaA + lambdaB, λ₂ = lambdaA − lambdaB.
    */
   public static double[] computeLambdaAB(MotorThermalParameters p, double current)
   {
      double R0  = p.getWindingResistanceAtReferenceTemperature();
      double Ch  = p.getHousingThermalCapacitance();
      double Cw  = p.getWindingThermalCapacitance();
      double Rwh = p.getWindingToHousingThermalResistance();
      double Rha = p.getHousingToAmbientThermalResistance();
      double ic2 = current * current;
      double A = Ch * Rha * (ic2 * R0 * Rwh * ALPHA_CU - 1.0);
      double B = Cw * (Rha + Rwh);
      double denom = 2.0 * Ch * Cw * Rwh * Rha;
      double lambdaA = (A - B) / denom;
      double lambdaB = Math.sqrt((A + B) * (A + B) + 4.0 * Ch * Cw * Rha * Rha) / denom;
      return new double[] {lambdaA, lambdaB};
   }

   // -------------------------------------------------------------------------
   // Analytical temperature solution
   // -------------------------------------------------------------------------

   /**
    * Winding temperature [°C] at time {@code t} seconds after applying {@code current} A, given
    * initial winding temperature {@code Tw0} and initial housing temperature {@code Th0}.
    *
    * <p>Uses the closed-form matrix exponential solution of the linearized 2×2 ODE.
    *
    * @param p          actuator thermal parameters
    * @param current    applied current [A]
    * @param ambientTemp ambient (environment) temperature [°C]
    * @param Tw0        initial winding temperature [°C]
    * @param Th0        initial housing temperature [°C]
    * @param t          elapsed time [s]
    * @return winding temperature at time t [°C]
    */
   public static double evaluateWindingTemperature(MotorThermalParameters p, double current,
                                                   double ambientTemp, double Tw0, double Th0, double t)
   {
      double[] coefficients = computeExponentialCoefficients(p, current, ambientTemp, Tw0, Th0);
      double Tssw    = coefficients[0];
      double lambda1 = coefficients[1];
      double lambda2 = coefficients[2];
      double c1      = coefficients[3];
      double c2      = coefficients[4];
      double a22     = computeA22(p);
      return c1 * (lambda1 - a22) * Math.exp(lambda1 * t) + c2 * (lambda2 - a22) * Math.exp(lambda2 * t) + Tssw;
   }

   /**
    * Housing temperature [°C] at time {@code t} seconds after applying {@code current} A, given
    * initial conditions.
    */
   public static double evaluateHousingTemperature(MotorThermalParameters p, double current,
                                                   double ambientTemp, double Tw0, double Th0, double t)
   {
      double[] coefficients = computeExponentialCoefficients(p, current, ambientTemp, Tw0, Th0);
      double Tssh    = coefficients[5];
      double lambda1 = coefficients[1];
      double lambda2 = coefficients[2];
      double c1      = coefficients[3];
      double c2      = coefficients[4];
      double a21     = computeA21(p);
      return c1 * a21 * Math.exp(lambda1 * t) + c2 * a21 * Math.exp(lambda2 * t) + Tssh;
   }

   /**
    * Returns the coefficients of the matrix exponential solution:
    * {@code [Tssw, lambda1, lambda2, c1, c2, Tssh]}.
    *
    * <p>The winding and housing temperatures are:
    * <pre>
    *   T_w(t) = c1*(λ₁−a₂₂)*exp(λ₁·t) + c2*(λ₂−a₂₂)*exp(λ₂·t) + Tssw
    *   T_h(t) = c1*a₂₁*exp(λ₁·t)       + c2*a₂₁*exp(λ₂·t)       + Tssh
    * </pre>
    */
   public static double[] computeExponentialCoefficients(MotorThermalParameters p, double current,
                                                         double ambientTemp, double Tw0, double Th0)
   {
      double Tssw    = computeSteadyStateWindingTemp(p, current, ambientTemp);
      double Tssh    = computeSteadyStateHousingTemp(p, current, ambientTemp);
      double[] lAB   = computeLambdaAB(p, current);
      double lambda1 = lAB[0] + lAB[1];
      double lambda2 = lAB[0] - lAB[1];
      double a21     = computeA21(p);
      double a22     = computeA22(p);
      double dLambda = lambda1 - lambda2;

      // Solve for c1, c2 from initial conditions [T_w0 - Tssw; T_h0 - Tssh] = V * [c1; c2]
      // where V = [(λ₁−a₂₂), (λ₂−a₂₂); a₂₁, a₂₁]
      double dw = Tw0 - Tssw;
      double dh = Th0 - Tssh;
      double c1 = (a21 * dw - (lambda2 - a22) * dh) / (a21 * dLambda);
      double c2 = ((lambda1 - a22) * dh - a21 * dw) / (a21 * dLambda);

      return new double[] {Tssw, lambda1, lambda2, c1, c2, Tssh};
   }

   // -------------------------------------------------------------------------
   // Fuse curve (time-to-failure)
   // -------------------------------------------------------------------------

   /**
    * Computes the maximum duration [s] a constant {@code current} can be applied before the
    * winding temperature reaches {@link MotorThermalParameters#getMaxWindingTemperature()},
    * assuming both nodes start at ambient temperature.
    *
    * @param p           actuator thermal parameters
    * @param current     applied current [A]; must exceed steady-state max for a finite result
    * @param ambientTemp ambient (initial) temperature [°C]
    * @return time-to-failure [s]
    */
   public static double computeMaxPeakDuration(MotorThermalParameters p, double current, double ambientTemp)
   {
      double Tmax = p.getMaxWindingTemperature();
      double[] coefficients = computeExponentialCoefficients(p, current, ambientTemp, ambientTemp, ambientTemp);
      double Tssw    = coefficients[0];
      double lambda1 = coefficients[1];
      double lambda2 = coefficients[2];
      double c1      = coefficients[3];
      double c2      = coefficients[4];
      double a22     = computeA22(p);
      double K1      = c1 * (lambda1 - a22);
      double K2      = c2 * (lambda2 - a22);
      return bisect(lambda1, lambda2, K1, K2, Tssw, Tmax);
   }

   /**
    * Generates the full fuse curve for the given ambient temperature.
    *
    * @param p               actuator thermal parameters
    * @param ambientTemp     ambient temperature [°C]
    * @param minPeakDuration iteration stops when time-to-failure falls below this [s]
    * @return 2D array where each row is {@code [current [A], maxPeakDuration [s]]}
    */
   public static double[][] computeFuseCurve(MotorThermalParameters p, double ambientTemp, double minPeakDuration)
   {
      int iStart = (int) Math.ceil(computeSteadyStateMaxCurrent(p, ambientTemp));
      List<double[]> fuseCurve = new ArrayList<>();

      for (int ic = iStart; ; ic++)
      {
         double t = computeMaxPeakDuration(p, ic, ambientTemp);
         if (t < minPeakDuration)
            break;
         fuseCurve.add(new double[] {ic, t});
      }

      return fuseCurve.toArray(new double[0][]);
   }

   // -------------------------------------------------------------------------
   // Internal helpers
   // -------------------------------------------------------------------------

   private static double bisect(double lambda1, double lambda2, double K1, double K2, double Tssw, double Tmax)
   {
      double lo = 0.0;
      double hi = BISECTION_UPPER_BOUND_SECONDS;

      for (int iter = 0; iter < BISECTION_ITERATIONS; iter++)
      {
         double mid  = 0.5 * (lo + hi);
         double fMid = K1 * Math.exp(lambda1 * mid) + K2 * Math.exp(lambda2 * mid) + Tssw - Tmax;
         if (fMid < 0.0)
            lo = mid;
         else
            hi = mid;
      }

      return 0.5 * (lo + hi);
   }
}
