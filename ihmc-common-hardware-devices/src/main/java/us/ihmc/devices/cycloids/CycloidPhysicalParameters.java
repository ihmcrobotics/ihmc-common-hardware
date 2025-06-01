package us.ihmc.devices.cycloids;

import us.ihmc.xmlDescription.devices.parameters.XmlCycloidPhysicalParameters;

public class CycloidPhysicalParameters
{
   private int countsPerMotorRevolution;
   private int countsPerOutputRevolution;
   private double gearRatio;
   private double kt;

   public CycloidPhysicalParameters()
   {
   }

   public CycloidPhysicalParameters(XmlCycloidPhysicalParameters physicalParameters)
   {
      this(physicalParameters.getInputResolution(), physicalParameters.getOutputResolution(), physicalParameters.getKt(), physicalParameters.getGearRatio());
   }

   public CycloidPhysicalParameters(int countsPerMotorRevolution, int countsPerOutputRevolution, double gearRatio, double kt)
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
}