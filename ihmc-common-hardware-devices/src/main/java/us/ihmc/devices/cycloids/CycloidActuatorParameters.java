package us.ihmc.devices.cycloids;

public class CycloidActuatorParameters
{
   private int countsPerMotorRevolution;
   private int countsPerOutputRevolution;
   private double gearRatio;
   private double kt;

   public CycloidActuatorParameters()
   {
   }

   public CycloidActuatorParameters(int countsPerMotorRevolution, int countsPerOutputRevolution, double gearRatio, double kt)
   {
      setCountsPerMotorRevolution(countsPerMotorRevolution);
      setCountsPerOutputRevolution(countsPerOutputRevolution);
      setGearRatio(gearRatio);
      setKt(kt);
   }

   public int getCountsPerMotorRevolution()
   {
      return countsPerMotorRevolution;
   }

   public void setCountsPerMotorRevolution(int countsPerMotorRevolution)
   {
      this.countsPerMotorRevolution = countsPerMotorRevolution;
   }

   public int getCountsPerOutputRevolution()
   {
      return countsPerOutputRevolution;
   }

   public void setCountsPerOutputRevolution(int countsPerOutputRevolution)
   {
      this.countsPerOutputRevolution = countsPerOutputRevolution;
   }

   public double getGearRatio()
   {
      return gearRatio;
   }

   public void setGearRatio(double gearRatio)
   {
      this.gearRatio = gearRatio;
   }

   public double getKt()
   {
      return kt;
   }

   public void setKt(double kt)
   {
      this.kt = kt;
   }

   public static CycloidActuatorParameters createCycloidParameters(CycloidActuatorPackage actuatorPackage)
   {
      switch (actuatorPackage)
      {
         case A:
         case B:
         case C:
         case D:
            return creatCycloidParameters();
         default:
            throw new IllegalArgumentException("Unexpected actuator package for cycloid parameters: " + actuatorPackage);
      }
   }

   public static CycloidActuatorParameters creatCycloidParameters()
   {
      CycloidActuatorParameters parameters = new CycloidActuatorParameters();
      parameters.setCountsPerMotorRevolution(1 << 17);
      parameters.setCountsPerOutputRevolution(1 << 17);
      parameters.setGearRatio(17.0);
      parameters.setKt(0.073);
      return parameters;
   }
}