package us.ihmc.hardwareXMLToolkit.devices;

import jakarta.xml.bind.annotation.*;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlEtherSnacksBoard")
public class XmlEtherSnacksBoard extends AbstractXmlEtherCATDevice
{
   @XmlElement(required = true)
   protected XmlDaughterDevices daughterDevices;

   @XmlElement(required = true)
   protected XmlEtherSnacksBoardType boardType;

   @XmlElement(defaultValue = "1539") //1539 in decimal = 0x00000603 in hex
   protected int vendorID = 0x00000603;

   @XmlElement(defaultValue = "268448003") //268448003 in decimal = 0x10003103 in hex
   protected int productCode = 0x10003103;

   @XmlElementWrapper(name = "pdoList")
   @XmlElement(name = "XmlPDOType")
   protected List<XmlPDOType> pdoList;

   @XmlElementWrapper(name = "sdoList")
   @XmlElement(name = "XmlSDOType")
   protected List<XmlSDOType> sdoList;

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

   public List<XmlPDOType> getPDOList()
   {
      if (pdoList == null)
         pdoList = new ArrayList<>();
      return pdoList;
   }

   public List<XmlSDOType> getSDOList()
   {
      if (sdoList == null)
         sdoList = new ArrayList<>();
      return sdoList;
   }
}
