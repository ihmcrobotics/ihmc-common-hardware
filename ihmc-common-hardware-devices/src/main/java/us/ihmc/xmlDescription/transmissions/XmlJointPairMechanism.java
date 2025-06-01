package us.ihmc.xmlDescription.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.xmlDescription.devices.AbstractXmlDevice;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlJointPairMechanism")
public class XmlJointPairMechanism extends AbstractXmlDevice
{
   @XmlElement(required = true)
   protected String robotSide;

   @XmlElement(required = true)
   protected String pitchJoint;
   @XmlElement(required = true)
   protected String rollJoint;

   @XmlElement(required = true)
   private String leftMotor;
   @XmlElement
   private String rightMotor;

   @XmlElement
   protected double pitchJointOffset;
   @XmlElement
   protected double rollJointOffset;

   public String getRobotSide()
   {
      return robotSide;
   }

   public String getPitchJoint()
   {
      return pitchJoint;
   }

   public String getRollJoint()
   {
      return rollJoint;
   }

   public String getLeftMotor()
   {
      return leftMotor;
   }

   public String getRightMotor()
   {
      return rightMotor;
   }

   public double getPitchJointOffset()
   {
      return pitchJointOffset;
   }

   public double getRollJointOffset()
   {
      return rollJointOffset;
   }
}
