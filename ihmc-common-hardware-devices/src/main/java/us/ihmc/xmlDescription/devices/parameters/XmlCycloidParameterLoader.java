package us.ihmc.xmlDescription.devices.parameters;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import java.io.InputStream;

public class XmlCycloidParameterLoader
{
   private static final String parameterDirectory = "/parameters/actuators/";

   public static XmlCycloidParameters getCycloidParametersFromActuatorPackageName(String actuatorPackage)
   {
      InputStream parameterStream = XmlCycloidParameterLoader.class.getResourceAsStream(parameterDirectory + actuatorPackage + ".xml");
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(XmlCycloidParameters.class);
         Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

         return (XmlCycloidParameters) unmarshaller.unmarshal(parameterStream);
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      String actuatorPackage = "actuatorExample";

      XmlCycloidParameters cycloidParameters = getCycloidParametersFromActuatorPackageName(actuatorPackage);
      XmlCycloidPhysicalParameters physicalParameters = cycloidParameters.getPhysicalParameters();
      XmlCycloidSILParameters silParameters = cycloidParameters.getSilParameters();
      System.out.println("kt: " + physicalParameters.getKt());
      System.out.println("Gear Ratio: " + physicalParameters.getGearRatio());
      System.out.println("Input Resolution: " + physicalParameters.getInputResolution());
      System.out.println("Output Resolution: " + physicalParameters.getOutputResolution());
      System.out.println("Friction Gain: " + silParameters.getFrictionGain());
      System.out.println("Friction Slope: " + silParameters.getFrictionSlope());
      System.out.println("Damping Gain: " + silParameters.getDampingGain());
      System.out.println("Dahl Scalar: " + silParameters.getDahlScalar());
      System.out.println("Cogging Scalar: " + silParameters.getCoggingScalar());
      System.out.println("Acceleration/Integration Scalar: " + silParameters.getAccelerationIntegrationScalar());


   }
}
