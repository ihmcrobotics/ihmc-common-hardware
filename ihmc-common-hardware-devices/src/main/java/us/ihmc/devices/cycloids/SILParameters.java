package us.ihmc.devices.cycloids;

public class SILParameters
{
   private double dahlFrictionForceGain;
   private double dahlFrictionSlope;
   private double linearDampingCompensationGain;
   private double dahlOutputScalar;
   private double linearDampingOutputScalar;
   private double coggingOutputScalar;
   private double accelerationIntegrationScalar;

   public double getDahlFrictionForceGain()
   {
      return dahlFrictionForceGain;
   }

   public void setDahlFrictionForceGain(double dahlFrictionForceGain)
   {
      this.dahlFrictionForceGain = dahlFrictionForceGain;
   }

   public double getDahlFrictionSlope()
   {
      return dahlFrictionSlope;
   }

   public void setDahlFrictionSlope(double dahlFrictionSlope)
   {
      this.dahlFrictionSlope = dahlFrictionSlope;
   }

   public double getLinearDampingCompensationGain()
   {
      return linearDampingCompensationGain;
   }

   public void setLinearDampingCompensationGain(double linearDampingCompensationGain)
   {
      this.linearDampingCompensationGain = linearDampingCompensationGain;
   }

   public double getDahlOutputScalar()
   {
      return dahlOutputScalar;
   }

   public void setDahlOutputScalar(double dahlOutputScalar)
   {
      this.dahlOutputScalar = dahlOutputScalar;
   }

   public double getLinearDampingOutputScalar()
   {
      return linearDampingOutputScalar;
   }

   public void setLinearDampingOutputScalar(double linearDampingOutputScalar)
   {
      this.linearDampingOutputScalar = linearDampingOutputScalar;
   }

   public double getCoggingOutputScalar()
   {
      return coggingOutputScalar;
   }

   public void setCoggingOutputScalar(double coggingOutputScalar)
   {
      this.coggingOutputScalar = coggingOutputScalar;
   }

   public double getAccelerationIntegrationScalar()
   {
      return accelerationIntegrationScalar;
   }

   public void setAccelerationIntegrationScalar(double accelerationIntegrationScalar)
   {
      this.accelerationIntegrationScalar = accelerationIntegrationScalar;
   }

   /**
    * Create the SIL Parameters for the specific actuator package
    *
    * @param actuatorPackage Specific actuator package being used
    * @return The SILParameters object for the actuator package
    */
   public static SILParameters createParameters(CycloidActuatorPackage actuatorPackage)
   {
      return switch (actuatorPackage)
      {
         case A, B, C, D -> createParameters();
         default -> throw new IllegalArgumentException("Unexpected actuator package for SIL parameters: " + actuatorPackage);
      };
   }

   /**
    * @return The SILParameters object for AP01
    */
   private static SILParameters createParameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.4);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }
}