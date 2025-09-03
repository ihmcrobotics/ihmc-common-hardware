package us.ihmc.hardwareXMLToolkit;

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

   /**
    * @return if the object is present and needs to be created
    */
   public boolean isPresent()
   {
      return present;
   }

   /**
    *  Set if the object should be present or not
    * @param present present if true, not present if false
    */
   public void setPresent(boolean present)
   {
      this.present = present;
   }
}
