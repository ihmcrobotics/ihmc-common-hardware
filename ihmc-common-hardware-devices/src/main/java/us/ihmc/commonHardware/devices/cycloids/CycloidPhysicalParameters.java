package us.ihmc.commonHardware.devices.cycloids;

import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlCycloidPhysicalParameters;

/**
 * This class defines the physical parameters of a cycloid, including the motor and output enocder revolution counts
 */
public class CycloidPhysicalParameters
{
   private int countsPerMotorRevolution;
   private int countsPerOutputRevolution;
   private double gearRatio;
   private double kt;

   /**
    * Construct the physical parameters from an xml
    *
    * @param physicalParameters Holds the physical parameters of the cycloid from an xml
    */
   public CycloidPhysicalParameters(XmlCycloidPhysicalParameters physicalParameters)
   {
      this(physicalParameters.getInputResolution(), physicalParameters.getOutputResolution(), physicalParameters.getGearRatio(), physicalParameters.getKt());
   }

   /**
    * Construct the physical parameters using given parameters
    *
    * @param inputResolution  The number of bits the input encoder uses
    * @param outputResolution The number of bits the output encoder uses
    * @param gearRatio        Gear ratio of the actuator
    * @param kt               ratio of current to torque for the motor
    */
   public CycloidPhysicalParameters(int inputResolution, int outputResolution, double gearRatio, double kt)
   {
      setCountsPerMotorRevolution(1 << inputResolution);
      setCountsPerOutputRevolution(1 << outputResolution);
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