package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlH4EtherCATJunctionPort")
public class XmlH4EtherCATJunctionPort extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected int junctionPort;

   public int getJunctionPort()
   {
      return junctionPort;
   }
}
