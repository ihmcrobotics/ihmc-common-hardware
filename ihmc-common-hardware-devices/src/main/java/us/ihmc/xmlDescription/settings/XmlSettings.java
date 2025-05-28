package us.ihmc.xmlDescription.settings;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlSettings")
public class XmlSettings
{
   @XmlElements({@XmlElement(name = "XmlEtherCATSettings", type = XmlEtherCATSettings.class),
         @XmlElement(name = "XmlRosSettings", type = XmlRosSettings.class)})
   protected List<Object> settings;

   public List<Object> getSettings()
   {
      if (settings == null)
      {
         settings = new ArrayList<Object>();
      }
      return this.settings;
   }

}
