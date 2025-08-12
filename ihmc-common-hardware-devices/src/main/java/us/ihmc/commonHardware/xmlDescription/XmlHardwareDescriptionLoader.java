package us.ihmc.commonHardware.xmlDescription;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 * This class is used to load the xml files that describe the hardware specifications of the robot
 *
 * @author Reese Peterson
 */
public class XmlHardwareDescriptionLoader
{
   public static final File descriptionFileDirectory = new File(System.getProperty("user.home"), "robot-configuration");
   private static final String DEFAULT_DIRECTORY = "parameters/";

   /**
    * Get the hardware description of the robot from the xml files in the provided directory
    *
    * @param directory String representation of the directory the xml files are in
    * @param names Collection of names of the xml files to get hardware descriptions from
    * @return A list of {@code XmlHardwareDescription} objects representing each xml file provided
    */
   public static List<XmlHardwareDescription> getHardwareDescriptionFromAlternateResources(String directory, Collection<String> names)
   {
      List<XmlHardwareDescription> hardwareDescriptions = new ArrayList<>();
      for(String resource : names)
         hardwareDescriptions.add(getHardwareDescriptionFromAlternateResource(directory, resource));
      return hardwareDescriptions;
   }

   /**
    * Get the hardware description of the robot from the xml files in the default directory
    *
    * @param names Collection of names of the xml files to get hardware descriptions from
    * @return A list of {@code XmlHardwareDescription} objects representing each xml file provided
    */
   public static List<XmlHardwareDescription> getHardwareDescriptionFromResources(Collection<String> names)
   {
      return names.stream().map(XmlHardwareDescriptionLoader::getHardwareDescriptionFromResources).toList();
   }

   public static XmlHardwareDescription getHardwareDescriptionFromAlternateResource(String directory, String name)
   {
      InputStream hardwareStream = XmlHardwareDescriptionLoader.class.getClassLoader().getResourceAsStream(directory + name);
      XmlHardwareDescription hardwareDescription = XmlHardwareDescriptionLoader.getHardwareDescription(hardwareStream);
      return hardwareDescription;
   }

   /**
    * @param descriptionFile xml file to be read
    * @return An {@code XmlHardwareDescription} object based on the information in the xml file provided
    */
   public static XmlHardwareDescription getHardwareDescription(File descriptionFile)
   {
      if (!descriptionFileDirectory.exists() || !descriptionFileDirectory.isDirectory())
      {
         throw new RuntimeException("Cannot load robot configuration xml from " + descriptionFileDirectory.getAbsolutePath() + "." + System.lineSeparator());
      }

      try
      {
         InputStream is = new FileInputStream(descriptionFile);
         return getHardwareDescription(is);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Get the hardware description of the robot from the xml file in the default directory
    *
    * @param name Name of the xml file to get hardware descriptions from
    * @return An {@code XmlHardwareDescription} representing the xml file provided
    */
   public static XmlHardwareDescription getHardwareDescriptionFromResources(String name)
   {
      return getHardwareDescriptionFromAlternateResource(DEFAULT_DIRECTORY, name);
   }

   /**
    * Get the hardware description from an input stream
    *
    * @param is input stream of xml info
    * @return An {@code XmlHardwareDescription} representing the xml file provided
    */
   public static XmlHardwareDescription getHardwareDescription(InputStream is)
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(XmlHardwareDescription.class);
         Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

         XmlHardwareDescription hardware = (XmlHardwareDescription) unmarshaller.unmarshal(is);

         return hardware;
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }
}
