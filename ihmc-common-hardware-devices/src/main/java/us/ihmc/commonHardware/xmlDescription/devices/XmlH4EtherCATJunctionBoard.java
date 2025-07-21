package us.ihmc.commonHardware.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;

import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlH4EtherCATJunctionBoard")
public class XmlH4EtherCATJunctionBoard extends AbstractXmlDevice
{
   @XmlElements({@XmlElement(name = "XmlH4EtherCATJunction", type = XmlH4EtherCATJunctionPort.class)})
   protected List<XmlH4EtherCATJunctionPort> junctions;

   public List<XmlH4EtherCATJunctionPort> getJunctions()
   {
      return junctions;
   }
}
