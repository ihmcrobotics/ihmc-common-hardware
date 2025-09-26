package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlDaughterDevices")
public class XmlDaughterDevices
{
   @XmlElements({@XmlElement(name = "XmlIMU", type = XmlIMU.class),
                 @XmlElement(name = "XmlTemperatureSensor", type = XmlTemperatureSensor.class)})
   protected List<AbstractXmlDevice> devices;

   public List<AbstractXmlDevice> getDevices()
   {
      if (devices == null)
         devices = new ArrayList<>();
      return devices;
   }
}
