package us.ihmc.hardwareXMLToolkit.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "CycloidPhysicalParameters")
public class XmlCycloidPhysicalParameters
{
   @XmlElement(required = true)
   private double kt;

   @XmlElement(required = true)
   private double gearRatio;

   @XmlElement(required = true)
   private int inputResolution;

   @XmlElement(required = true)
   private int outputResolution;

   public double getKt()
   {
      return kt;
   }

   public double getGearRatio()
   {
      return gearRatio;
   }

   public int getInputResolution()
   {
      return inputResolution;
   }

   public int getOutputResolution()
   {
      return outputResolution;
   }
}
