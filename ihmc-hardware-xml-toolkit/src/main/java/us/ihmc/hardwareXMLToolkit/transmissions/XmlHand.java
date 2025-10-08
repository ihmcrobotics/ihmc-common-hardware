package us.ihmc.hardwareXMLToolkit.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.hardwareXMLToolkit.devices.AbstractXmlDevice;

import java.util.ArrayList;
import java.util.List;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlHand")
public class XmlHand extends AbstractXmlDevice
{
   @XmlElement(required = true)
   protected String side;

   @XmlElement(required = true)
   protected String type;

   @XmlElement(required = true)
   protected int parentBoardAlias;

   @XmlElement(required = true, name = "joint")
   protected List<String> joints;

   public String getSide()
   {
      return side;
   }

   public String getType()
   {
      return type;
   }

   public int getParentBoardAlias()
   {
      return parentBoardAlias;
   }

   public List<String> getJoints()
   {
      if (joints == null)
         joints = new ArrayList<>();

      return joints;
   }
}
