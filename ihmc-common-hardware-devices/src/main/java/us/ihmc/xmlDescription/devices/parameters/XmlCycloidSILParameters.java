package us.ihmc.xmlDescription.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "CycloidSILParameters")
public class XmlCycloidSILParameters
{
   @XmlElement(required = true)
   private double frictionGain;

   @XmlElement(required = true)
   private double frictionSlope;

   @XmlElement(required = true)
   private double dampingGain;

   @XmlElement(defaultValue = "1.0")
   private double dahlScalar = 1.0;

   @XmlElement(defaultValue = "1.0")
   private double coggingScalar = 1.0;

   @XmlElement(defaultValue = "1.0")
   private double accelerationIntegrationScalar = 1.0;

   public double getFrictionGain()
   {
      return frictionGain;
   }

   public double getFrictionSlope()
   {
      return frictionSlope;
   }

   public double getDampingGain()
   {
      return dampingGain;
   }

   public double getDahlScalar()
   {
      return dahlScalar;
   }

   public double getCoggingScalar()
   {
      return coggingScalar;
   }

   public double getAccelerationIntegrationScalar()
   {
      return accelerationIntegrationScalar;
   }
}
