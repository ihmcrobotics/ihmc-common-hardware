package us.ihmc.xmlDescription.settings;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEtherCATSettings")
public class XmlEtherCATSettings
{
   @XmlElement(required = true)
   protected String ethercatInterface;

   public XmlEtherCATSettings()
   {
   }
   
   public XmlEtherCATSettings(String ethercatInterface)
   {
      this.ethercatInterface = ethercatInterface;
   }
   
   public String getEthercatInterface()
   {
      return ethercatInterface;
   }
   
   public void setEthercatInterface(String ethercatInterface)
   {
      this.ethercatInterface = ethercatInterface;
   }

}
