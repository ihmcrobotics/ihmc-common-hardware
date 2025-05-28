package us.ihmc.xmlDescription.devices;

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
   @XmlElements({@XmlElement(name = "XmlH4EtherCATJunctionBoard", type = XmlH4EtherCATJunctionBoard.class),
                 @XmlElement(name = "XmlEtherSnacksTDKIMU", type = XmlEtherSnacksBoard.class),
                 @XmlElement(name = "XmlIMU", type = XmlIMU.class),
                 @XmlElement(name = "XmlPlatinumTwitter", type = XmlPlatinumTwitter.class),})
   protected List<? extends AbstractXmlDevice> devices;

   public List<? extends AbstractXmlDevice> getDevices()
   {
      if (devices == null)
      {
         devices = new ArrayList<>();
      }
      return this.devices;
   }
}
