package us.ihmc.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlElement;

public class XmlTemperatureSensor extends AbstractXmlDevice
{
   @XmlElement(defaultValue = "1.0")
   private double conversionScale;

   @XmlElement(defaultValue = "0.0")
   private double conversionOffset;

   public double getConversionScale()
   {
      return conversionScale;
   }

   public double getConversionOffset()
   {
      return conversionOffset;
   }
}
