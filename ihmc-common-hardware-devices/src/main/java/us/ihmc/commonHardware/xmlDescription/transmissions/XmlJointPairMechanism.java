package us.ihmc.commonHardware.xmlDescription.transmissions;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.commonHardware.xmlDescription.devices.AbstractXmlDevice;

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

   @XmlElement(defaultValue = "0.0")
   private double leftMotorOffset;
   @XmlElement(defaultValue = "0.0")
   private double rightMotorOffset;

   @XmlElement(defaultValue = "0.0")
   protected double pitchJointOffset;
   @XmlElement(defaultValue = "0.0")
   protected double rollJointOffset;

   @XmlElement(defaultValue = "1.0")
   protected double rollSign;
   @XmlElement(defaultValue = "1.0")
   protected double pitchSign;

   @XmlElement(defaultValue = "Double.POSITIVE_INFINITY")
   protected double torqueBreakFrequency = Double.POSITIVE_INFINITY;

   public double getRollJointLowerLimit()
   {
      return rollJointLowerLimit;
   }

   public double getRollJointUpperLimit()
   {
      return rollJointUpperLimit;
   }

   public double getPitchJointLowerLimit()
   {
      return pitchJointLowerLimit;
   }

   public double getPitchJointUpperLimit()
   {
      return pitchJointUpperLimit;
   }

   @XmlElement
   private double rollJointLowerLimit;
   @XmlElement
   private double rollJointUpperLimit;
   @XmlElement
   private double pitchJointLowerLimit;
   @XmlElement
   private double pitchJointUpperLimit;


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

   public double getLeftMotorOffset()
   {
      return leftMotorOffset;
   }

   public double getRightMotorOffset()
   {
      return rightMotorOffset;
   }

   public double getTorqueBreakFrequency()
   {
      return torqueBreakFrequency;
   }

   public double getPitchSign()
   {
      return pitchSign;
   }

   public double getRollSign()
   {
      return rollSign;
   }
}
