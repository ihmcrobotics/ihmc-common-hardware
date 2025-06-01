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
         case A001 -> createA001Parameters();
         case A002 -> createA002Parameters();
         case A009 -> createA009Parameters();
         case A013 -> createA013Parameters();
         case A005 -> createA005Parameters();
         case A011 -> createA011Parameters();
         case A010 -> createA010Parameters();
         case A004 -> createA004Parameters();
         case A003 -> createA003Parameters();
         case A012 -> createA012Parameters();
         case A008 -> createA008Parameters();
         case A006 -> createA006Parameters();
         case A007 -> createA007Parameters();
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
   private static SILParameters createA001Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.003);
      parameters.setDahlFrictionSlope(100.0);
      parameters.setLinearDampingCompensationGain(0.0005);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA002Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.003);
      parameters.setDahlFrictionSlope(100.0);
      parameters.setLinearDampingCompensationGain(0.0007);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA009Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.05);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0005);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA013Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.05);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0005);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA005Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.05);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0005);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA011Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.1);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.001);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA010Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.1);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.001);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA004Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.1);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.001);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA003Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.1);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.001);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA012Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.03);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0006);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA008Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.03);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0006);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA006Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.03);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0007);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }

   private static SILParameters createA007Parameters()
   {
      SILParameters parameters = new SILParameters();
      parameters.setDahlFrictionForceGain(0.03);
      parameters.setDahlFrictionSlope(200.0);
      parameters.setLinearDampingCompensationGain(0.0007);
      parameters.setCoggingOutputScalar(1.0);
      parameters.setDahlOutputScalar(1.0);
      parameters.setLinearDampingOutputScalar(1.0);
      parameters.setAccelerationIntegrationScalar(0.0);

      return parameters;
   }
}