package us.ihmc.devices.cycloids;

import us.ihmc.xmlDescription.devices.parameters.XmlCycloidSILParameters;

public class CycloidSILParameters
{
   private double dahlFrictionForceGain;
   private double dahlFrictionSlope;
   private double linearDampingCompensationGain;
   private double dahlOutputScalar;
   private double linearDampingOutputScalar;
   private double coggingOutputScalar;
   private double accelerationIntegrationScalar;

   public CycloidSILParameters()
   {

   }

   public CycloidSILParameters(XmlCycloidSILParameters xmlCycloidSILParameters)
   {
      dahlFrictionForceGain = xmlCycloidSILParameters.getFrictionGain();
      dahlFrictionSlope = xmlCycloidSILParameters.getFrictionSlope();
      linearDampingCompensationGain = xmlCycloidSILParameters.getDampingGain();
      dahlOutputScalar = xmlCycloidSILParameters.getDahlScalar();
      coggingOutputScalar = xmlCycloidSILParameters.getCoggingScalar();
      accelerationIntegrationScalar = xmlCycloidSILParameters.getAccelerationIntegrationScalar();
   }

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
}