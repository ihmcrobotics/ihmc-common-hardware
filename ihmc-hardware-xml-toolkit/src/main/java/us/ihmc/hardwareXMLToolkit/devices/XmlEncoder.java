package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEncoder")
public class XmlEncoder extends AbstractXmlDevice
{
   @XmlElement(defaultValue = "false")
   protected boolean invertDirection;

   public boolean isInvertDirection()
   {
      return invertDirection;
   }
}
