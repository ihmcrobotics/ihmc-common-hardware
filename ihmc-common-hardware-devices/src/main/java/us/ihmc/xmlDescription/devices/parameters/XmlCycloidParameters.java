package us.ihmc.xmlDescription.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "CycloidParameters")
public class XmlCycloidParameters
{
   @XmlElement(required = true)
   private XmlCycloidPhysicalParameters physicalParameters;

   @XmlElement(required = true)
   private XmlCycloidSILParameters silParameters;

   public XmlCycloidPhysicalParameters getPhysicalParameters()
   {
      return physicalParameters;
   }

   public XmlCycloidSILParameters getSilParameters()
   {
      return silParameters;
   }
}
