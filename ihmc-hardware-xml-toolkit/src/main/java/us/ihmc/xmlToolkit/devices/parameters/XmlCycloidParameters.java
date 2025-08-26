package us.ihmc.xmlToolkit.devices.parameters;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlRootElement(name = "CycloidParameters")
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

   public void setPhysicalParameters(XmlCycloidPhysicalParameters physicalParameters)
   {
      this.physicalParameters = physicalParameters;
   }

   public void setSilParameters(XmlCycloidSILParameters silParameters)
   {
      this.silParameters = silParameters;
   }
}
