package us.ihmc.hardwareXMLToolkit.devices.parameters;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import java.io.InputStream;

public class XmlActuatorParameterLoader
{
   // Directory convention within any main/resources directory within the classpath of the runnable program (does not have to be in this project's resources)
   private static final String DEFAULT_ACTUATOR_PARAM_DIRECTORY = "xmlParameters/actuatorPackages/";

   // Directory convention within any main/resources directory within the classpath of the runnable program (does not have to be in this project's resources)
   private static final String DEFAULT_MOTOR_PARAM_DIRECTORY = "xmlParameters/motors/";

   public static XmlCycloidParameters getCycloidParametersFromActuatorPackageName(String parameterDirectory, String actuatorPackage)
   {
      InputStream parameterStream = XmlActuatorParameterLoader.class.getClassLoader().getResourceAsStream(parameterDirectory + actuatorPackage + ".xml");
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

   public static XmlCycloidParameters getCycloidParametersFromActuatorPackageName(String actuatorPackage)
   {
      return getCycloidParametersFromActuatorPackageName(DEFAULT_ACTUATOR_PARAM_DIRECTORY, actuatorPackage);
   }

   public static XmlMotorParameters getMotorParametersFromMotorName(String parameterDirectory, String actuatorPackage)
   {
      InputStream parameterStream = XmlActuatorParameterLoader.class.getClassLoader().getResourceAsStream(parameterDirectory + actuatorPackage + ".xml");
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(XmlMotorParameters.class);
         Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

         return (XmlMotorParameters) unmarshaller.unmarshal(parameterStream);
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static XmlMotorParameters getMotorParametersFromMotorName(String actuatorPackage)
   {
      return getMotorParametersFromMotorName(DEFAULT_MOTOR_PARAM_DIRECTORY, actuatorPackage);
   }

   public static void main(String[] args)
   {
      String exampleDirectory = "xmlExamples/parameters/";
      String exampleActuatorPackage = "ExampleCycloidParameters";

      XmlCycloidParameters cycloidParameters = getCycloidParametersFromActuatorPackageName(exampleDirectory, exampleActuatorPackage);
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
      System.out.println("Motor: " + cycloidParameters.getMotor());

      XmlMotorParameters motorParameters = getMotorParametersFromMotorName(exampleDirectory, cycloidParameters.getMotor());
      System.out.println("Manufacturer: " + motorParameters.getManufacturer());
      System.out.println("Model: " + motorParameters.getModel());
      System.out.println("kt: " + motorParameters.getKt());
      System.out.println("ResistanceLineToLine: " + motorParameters.getResistanceLineToLine());
      System.out.println("BusVoltage: " + motorParameters.getBusVoltage());
      System.out.println("MaxCurrentContinuous: " + motorParameters.getMaxCurrentContinuous());
      System.out.println("MaxCurrentPeak: " + motorParameters.getMaxCurrentPeak());
      System.out.println("MaxTorqueContinuous: " + motorParameters.getMaxTorqueContinuous());
      System.out.println("MaxTorquePeak: " + motorParameters.getMaxTorquePeak());
      System.out.println("MaxSpeed: " + motorParameters.getMaxSpeed());
      System.out.println("PolePairs: " + motorParameters.getPolePairs());
      System.out.println("Mass: " + motorParameters.getMass());
   }
}
