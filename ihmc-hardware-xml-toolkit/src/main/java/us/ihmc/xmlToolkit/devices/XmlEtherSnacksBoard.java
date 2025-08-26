package us.ihmc.xmlToolkit.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEtherSnacksBoard")
public class XmlEtherSnacksBoard extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected XmlDaughterDevices daughterDevices;

   @XmlElement(required = true)
   protected XmlEtherSnacksBoardType boardType;

   public List<AbstractXmlDevice> getDaughterDevices()
   {
      return this.daughterDevices.getDevices();
   }

   public XmlEtherSnacksBoardType getBoardType()
   {
      return boardType;
   }
}