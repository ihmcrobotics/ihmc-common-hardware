package us.ihmc.xmlToolkit.settings;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlRosSettings")
public class XmlRosSettings
{
   @XmlElement(required = true)
   protected int rosDomainId;

   public XmlRosSettings()
   {
   }
   
   public XmlRosSettings(int rosDomainId)
   {
      this.rosDomainId = rosDomainId;
   }
   
   public int getRosDomainId()
   {
      return rosDomainId;
   }

   public void setRosDomainId(int rosDomainId)
   {
      this.rosDomainId = rosDomainId;
   }

}
