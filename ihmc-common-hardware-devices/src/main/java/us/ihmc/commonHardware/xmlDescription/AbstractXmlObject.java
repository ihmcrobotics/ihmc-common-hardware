package us.ihmc.commonHardware.xmlDescription;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlObject")
public class AbstractXmlObject
{
   @XmlElement(required = true)
   protected boolean present;

   public boolean isPresent()
   {
      return present;
   }

   public void setPresent(boolean present)
   {
      this.present = present;
   }
}
