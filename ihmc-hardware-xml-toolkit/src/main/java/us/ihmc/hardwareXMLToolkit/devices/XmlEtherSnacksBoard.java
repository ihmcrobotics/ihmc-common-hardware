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
   protected int vendorID;

   @XmlElement(defaultValue = "0x10003103")
   protected int productCode;

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
