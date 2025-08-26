package us.ihmc.xmlToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.xmlToolkit.AbstractXmlObject;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlDevice")
public abstract class AbstractXmlDevice extends AbstractXmlObject
{
   @XmlElement(required = true)
   protected String name;

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }
}
