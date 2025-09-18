package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.XmlElement;

public class XmlHand extends AbstractXmlDevice
{
   @XmlElement(required = true)
   protected String handSide;

   @XmlElement(required = true)
   protected String handType;

   public String getHandSide()
   {
      return handSide;
   }

   public String getHandType()
   {
      return handType;
   }
}
