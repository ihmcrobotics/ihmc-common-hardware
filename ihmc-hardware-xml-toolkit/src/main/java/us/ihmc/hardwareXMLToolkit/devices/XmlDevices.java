package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlDevices")
public class XmlDevices
{
   @XmlElements({@XmlElement(name = "XmlH4EtherCATJunctionPort", type = XmlH4EtherCATJunctionPort.class),
                 @XmlElement(name = "XmlEtherSnacksBoard", type = XmlEtherSnacksBoard.class),
                 @XmlElement(name = "XmlIMU", type = XmlIMU.class),
                 @XmlElement(name = "XmlPlatinumTwitter", type = XmlPlatinumTwitter.class),
                 @XmlElement(name = "XmlAtiForceTorqueSensor", type = XmlAtiForceTorqueSensor.class),})
   protected List<AbstractXmlDevice> devices;

   public List<AbstractXmlDevice> getDevices()
   {
      if (devices == null)
      {
         devices = new ArrayList<>();
      }
      return this.devices;
   }
}
