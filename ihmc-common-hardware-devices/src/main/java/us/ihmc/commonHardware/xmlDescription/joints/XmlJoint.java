package us.ihmc.commonHardware.xmlDescription.joints;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlJoint")
public class XmlJoint
{
   @XmlElement(required = true)
   protected String name;

   @XmlElement(required = false)
   protected double jointencoderoffset;
   
   public XmlJoint()
   {
      
   }
   
   public XmlJoint(String name, double jointencoderoffset)
   {
      this.name = name;
      this.jointencoderoffset = jointencoderoffset;
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public double getJointencoderoffset()
   {
      return jointencoderoffset;
   }

   public void setJointencoderoffset(double jointencoderoffset)
   {
      this.jointencoderoffset = jointencoderoffset;
   }

}
