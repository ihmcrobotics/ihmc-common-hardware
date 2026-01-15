package us.ihmc.hardwareXMLToolkit.devices;

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

   @XmlElement(defaultValue = "0x00000603")
   protected int vendorID = 0x00000603;

   @XmlElement(defaultValue = "0x00000001")
   protected int productCode = 0x00000001;

   public List<AbstractXmlDevice> getDaughterDevices()
   {
      return this.daughterDevices.getDevices();
   }

   public XmlEtherSnacksBoardType getBoardType()
   {
      return boardType;
   }

   public int getVendorID()
   {
      return vendorID;
   }

   public int getProductCode()
   {
      return productCode;
   }
}
