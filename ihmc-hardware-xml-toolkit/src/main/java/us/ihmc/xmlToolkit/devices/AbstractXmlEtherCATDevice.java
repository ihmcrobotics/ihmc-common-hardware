package us.ihmc.xmlToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEtherCATDevice")
public abstract class AbstractXmlEtherCATDevice extends AbstractXmlDevice
{
   @XmlElement(required = true)
   protected int alias;
   @XmlElement(required = true)
   protected int position;

   public int getAlias()
   {
      return alias;
   }

   public void setAlias(int alias)
   {
      this.alias = alias;
   }

   public int getPosition()
   {
      return position;
   }

   public void setPosition(int position)
   {
      this.position = position;
   }

}
