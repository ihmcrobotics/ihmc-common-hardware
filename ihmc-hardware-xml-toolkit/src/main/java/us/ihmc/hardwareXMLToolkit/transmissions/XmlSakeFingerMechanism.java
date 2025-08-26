package us.ihmc.hardwareXMLToolkit.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.hardwareXMLToolkit.AbstractXmlObject;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlSakeFingerMechanism")
public class XmlSakeFingerMechanism extends AbstractXmlObject
{
   @XmlElement(required = true)
   protected String x1JointName;
   @XmlElement(required = true)
   protected String x2JointName;
   @XmlElement(required = true)
   protected String robotSide;

   public String getX1JointName()
   {
      return x1JointName;
   }
   public void setX1JointName(String name)
   {
      this.x1JointName = name;
   }

   public String getX2JointName()
   {
      return x2JointName;
   }
   public void setX2JointName(String name)
   {
      this.x2JointName = name;
   }
   public String getRobotSide()
   {
      return robotSide;
   }
   public void setRobotSide(String name)
   {
      this.robotSide = name;
   }
}
