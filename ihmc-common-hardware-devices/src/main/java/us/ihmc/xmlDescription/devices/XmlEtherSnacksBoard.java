package us.ihmc.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlElements;
import jakarta.xml.bind.annotation.XmlType;
import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEtherSnacksBoard")
public class XmlEtherSnacksBoard extends AbstractXmlEtherCATDevice
{
   @XmlElements({@XmlElement(name = "XmlIMU", type = XmlIMU.class), @XmlElement(name = "XmlTemperatureSensor", type = XmlTemperatureSensor.class)})
   protected List<AbstractXmlDevice> daughterDevices;

   @XmlElement(required = true)
   protected XmlEtherSnacksBoardType boardType;

   public List<AbstractXmlDevice> getDaughterDevices()
   {
      if (daughterDevices == null)
         daughterDevices = new ArrayList<AbstractXmlDevice>();
      return this.daughterDevices;
   }

   public XmlEtherSnacksBoardType getBoardType()
   {
      return boardType;
   }
}