package us.ihmc.commonHardware.xmlDescription.devices;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import us.ihmc.euclid.tuple3D.Vector3D;

@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "XmlAtiForceTorqueSensor")
public class XmlAtiForceTorqueSensor extends AbstractXmlEtherCATDevice
{
   @XmlElement(defaultValue = "0.0")
   protected double fxOffset;
   @XmlElement(defaultValue = "0.0")
   protected double fyOffset;
   @XmlElement(defaultValue = "0.0")
   protected double fzOffset;

   @XmlElement(defaultValue = "0.0")
   protected double txOffset;
   @XmlElement(defaultValue = "0.0")
   protected double tyOffset;
   @XmlElement(defaultValue = "0.0")
   protected double tzOffset;

   @XmlElement(defaultValue = "1.0")
   protected double fxScalar = 1.0;
   @XmlElement(defaultValue = "1.0")
   protected double fyScalar = 1.0;
   @XmlElement(defaultValue = "1.0")
   protected double fzScalar = 1.0;
   
   @XmlElement(defaultValue = "1.0")
   protected double txScalar = 1.0;
   @XmlElement(defaultValue = "1.0")
   protected double tyScalar = 1.0;
   @XmlElement(defaultValue = "1.0")
   protected double tzScalar = 1.0;

   public double getFxOffset()
   {
      return fxOffset;
   }

   public void setFxOffset(double fxOffset)
   {
      this.fxOffset = fxOffset;
   }

   public double getFyOffset()
   {
      return fyOffset;
   }

   public void setFyOffset(double fyOffset)
   {
      this.fyOffset = fyOffset;
   }

   public double getFzOffset()
   {
      return fzOffset;
   }

   public void setFzOffset(double fzOffset)
   {
      this.fzOffset = fzOffset;
   }

   public double getTxOffset()
   {
      return txOffset;
   }

   public void setTxOffset(double txOffset)
   {
      this.txOffset = txOffset;
   }

   public double getTyOffset()
   {
      return tyOffset;
   }

   public void setTyOffset(double tyOffset)
   {
      this.tyOffset = tyOffset;
   }

   public double getTzOffset()
   {
      return tzOffset;
   }

   public void setTzOffset(double tzOffset)
   {
      this.tzOffset = tzOffset;
   }
   
   public Vector3D getForceOffsets()
   {
      return new Vector3D(fxOffset, fyOffset, fzOffset);
   }

   public Vector3D getTorqueOffsets()
   {
      return new Vector3D(txOffset, tyOffset, tzOffset);
   }

   public Vector3D getForceScalar()
   {
      return new Vector3D(fxScalar, fyScalar, fzScalar);
   }

   public Vector3D getTorqueScalar()
   {
      System.out.println(txScalar + " " + tyScalar + " " + tzScalar);
      return new Vector3D(txScalar, tyScalar, tzScalar);
   }
}
