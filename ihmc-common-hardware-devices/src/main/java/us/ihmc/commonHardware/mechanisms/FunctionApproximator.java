package us.ihmc.commonHardware.mechanisms;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.ArrayList;
import java.util.List;

public class FunctionApproximator
{
   private static final int defaultHighestOrder = 6;

   private static final double defaultValueMatchingWeight = 1.0;
   private static final double defaultCoefficientRegularizationWeight = 0.0;

   private final double valueMatchingWeight;
   private final double coefficientRegularizationWeight;

   private final List<BasisFunction> basisFunctions = new ArrayList<>();
   private final DMatrixRMaj basisFunctionCoefficients = new DMatrixRMaj(0, 0);

   public FunctionApproximator()
   {
      this(defaultHighestOrder, true);
   }

   public FunctionApproximator(int highestOrder, boolean includeTrigonometric)
   {
      this(defaultValueMatchingWeight, defaultCoefficientRegularizationWeight, includeTrigonometric, highestOrder);
   }

   public FunctionApproximator(double valueMatchingWeight, double coefficientRegularizationWeight, boolean includeTrigonometric, int highestOrder)
   {
      this.valueMatchingWeight = valueMatchingWeight;
      this.coefficientRegularizationWeight = coefficientRegularizationWeight;

      for (int pow1 = 0; pow1 <= highestOrder; pow1++)
      {
         for (int pow2 = 0; pow2 <= highestOrder; pow2++)
         {
            int exponential1 = pow1;
            int exponential2 = pow2;
            basisFunctions.add((value1, value2) -> MathTools.pow(value1, exponential1) * MathTools.pow(value2, exponential2));
         }
      }

      if (includeTrigonometric)
      {
         // single variable trigonometric
         basisFunctions.add((value1, value2) -> Math.cos(value1));
         basisFunctions.add((value1, value2) -> Math.sin(value1));
         basisFunctions.add((value1, value2) -> Math.sin(value1) * Math.cos(value1));
         basisFunctions.add((value1, value2) -> Math.cos(value2));
         basisFunctions.add((value1, value2) -> Math.sin(value2));
         basisFunctions.add((value1, value2) -> Math.sin(value2) * Math.cos(value2));

         // multi variable trigonometric
         basisFunctions.add((value1, value2) -> Math.sin(value1) * Math.sin(value2));
         basisFunctions.add((value1, value2) -> Math.sin(value1) * Math.cos(value2));
         basisFunctions.add((value1, value2) -> Math.cos(value1) * Math.sin(value2));
         basisFunctions.add((value1, value2) -> Math.cos(value1) * Math.cos(value2));

         // single variable trigonometric inverse
         basisFunctions.add((value1, value2) -> Math.acos(value1));
         basisFunctions.add((value1, value2) -> Math.asin(value1));
         basisFunctions.add((value1, value2) -> Math.asin(value1) * Math.acos(value1));
         basisFunctions.add((value1, value2) -> Math.acos(value2));
         basisFunctions.add((value1, value2) -> Math.asin(value2));
         basisFunctions.add((value1, value2) -> Math.asin(value2) * Math.acos(value2));

         // multi variable trigonometric inverse
         basisFunctions.add((value1, value2) -> Math.asin(value1) * Math.asin(value2));
         basisFunctions.add((value1, value2) -> Math.asin(value1) * Math.acos(value2));
         basisFunctions.add((value1, value2) -> Math.acos(value1) * Math.asin(value2));
         basisFunctions.add((value1, value2) -> Math.acos(value1) * Math.acos(value2));
      }

   }

   private final DMatrixRMaj A = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj hessian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj b = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj reconstructedValues = new DMatrixRMaj(0, 0);
   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(0);

   private final DMatrixRMaj valueError = new DMatrixRMaj(0, 0);

   public void solveForCoefficients(double[] inputValue1, double[] inputValue2, double[] desiredFunctionValue)
   {
      int size = inputValue1.length;

      A.reshape(size, basisFunctions.size());
      b.reshape(size, 1);
      basisFunctionCoefficients.reshape(basisFunctions.size(), 1);

      for (int valueIndex = 0; valueIndex < size; valueIndex++)
      {
         for (int basisFunctionIndex = 0; basisFunctionIndex < basisFunctions.size(); basisFunctionIndex++)
         {
            A.set(valueIndex, basisFunctionIndex, basisFunctions.get(basisFunctionIndex).compute(inputValue1[valueIndex], inputValue2[valueIndex]));
         }

         b.set(valueIndex, 0, desiredFunctionValue[valueIndex]);
      }

      hessian.reshape(basisFunctions.size(), basisFunctions.size());
      jacobian.reshape(basisFunctions.size(), 1);
      CommonOps_DDRM.multInner(A, hessian);
      CommonOps_DDRM.multTransA(A, b, jacobian);

      CommonOps_DDRM.scale(valueMatchingWeight, hessian);
      CommonOps_DDRM.scale(valueMatchingWeight, jacobian);
      MatrixTools.addDiagonal(hessian, coefficientRegularizationWeight);

      solver.setA(hessian);
      solver.solve(jacobian, basisFunctionCoefficients);

      reconstructedValues.reshape(size, 1);
      valueError.reshape(size, 1);
      CommonOps_DDRM.mult(A, basisFunctionCoefficients, reconstructedValues);
      CommonOps_DDRM.subtract(b, reconstructedValues, valueError);

   }

   public double compute(double value1, double value2)
   {
      double functionValue = 0.0;
      for (int i = 0; i < basisFunctions.size(); i++)
         functionValue += basisFunctionCoefficients.get(i) * basisFunctions.get(i).compute(value1, value2);

      return functionValue;
   }


   private interface BasisFunction
   {
      double compute(double value1, double value2);
   }
}
