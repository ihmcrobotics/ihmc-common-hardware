package us.ihmc.hardwareStatusUI.visualizerSide;

import us.ihmc.hardwareStatusUI.controllerSide.DeviceStatusHolder;
import us.ihmc.hardwareXMLToolkit.devices.AbstractXmlDevice;
import us.ihmc.hardwareXMLToolkit.devices.XmlH4EtherCATJunctionPort;
import us.ihmc.hardwareXMLToolkit.devices.XmlIMU;
import us.ihmc.hardwareXMLToolkit.devices.XmlTemperatureSensor;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;
import java.util.ArrayList;
import java.util.List;

/**
 * Abstract class that parses a list of generic {@code AbstractXmlDevice}, and
 * creates a {@code UIDeviceStatusHolder} for each device in the list. In doing this,
 * this class searches a {@code YoVariableRegistry} for the device status YoVariables
 * corresponding to a given {@code AbstractXmlDevice} based on that xml device's name.
 * The specific list of {@code AbstractXmlDevice} to parse, and the details of how that
 * list is parsed is to be determined by the robot-specific child classes that extend
 * this one.
 */
public abstract class AbstractUIHardwareStatusManager
{
   /**
    * Enum that dictates if the device being added is a motor, a board holding multiple devices, or a single sensor
    */
   public enum DeviceType
   {MOTOR, BOARD, SENSOR}

   protected final ArrayList<UIDeviceStatusHolder> deviceDataHolders = new ArrayList<>();
   protected final SessionVisualizerControls sessionVisualizerControls;
   protected final SessionVisualizerToolkit toolkit;

   /**
    * Creates and manages all UI-side device status holders
    *
    * @param xmlDevices                Descriptions of all devices taken from xml descriptions of the robot
    * @param sessionVisualizerControls Controls for adding items to and controlling visualizer
    * @param toolkit                   Toolkit for easier control of the visualizer
    */
   public AbstractUIHardwareStatusManager(List<AbstractXmlDevice> xmlDevices,
                                          SessionVisualizerControls sessionVisualizerControls,
                                          SessionVisualizerToolkit toolkit)
   {
      this.sessionVisualizerControls = sessionVisualizerControls;
      this.toolkit = toolkit;

      sessionVisualizerControls.addSessionChangedListener((oldSession, newSession) ->
                                                          {
                                                             deviceDataHolders.clear();

                                                             if (newSession != null)
                                                             {
                                                                if (hasHardwareStatusVariables(xmlDevices))
                                                                   createDevices(xmlDevices);
                                                                else
                                                                   LogTools.info(
                                                                         "Skipping Hardware Status UI setup: connected session has no device status YoVariables.");
                                                             }
                                                          });
   }

   /**
    * Create instances for all the devices to be added to the UI
    *
    * @param xmlDevices Descriptions of all devices taken from xml descriptions of the robot
    */
   protected abstract void createDevices(List<AbstractXmlDevice> xmlDevices);

   /**
    * Create a data holder for an EtherSnacks device with daughter devices
    *
    * @param name                      Name of the device
    * @param description               Description of the device
    * @param position                  EtherCAT position
    * @param alias                     EtherCAT alias
    * @param daughterDevices           List of daughter devices
    * @param useParentPositionAndAlias If true, uses the alias of the parent device for all daughter devices
    * @param deviceType                Type of EtherCAT device being registered
    */
   protected void createNewEtherCATDataHolder(String name,
                                              String description,
                                              int position,
                                              int alias,
                                              List<AbstractXmlDevice> daughterDevices,
                                              boolean useParentPositionAndAlias,
                                              DeviceType deviceType)
   {
      createNewEtherCATDataHolder(name, description, position, alias, deviceType);

      for (AbstractXmlDevice daughterDevice : daughterDevices)
      {
         if (daughterDevice instanceof XmlIMU xmlIMU && useParentPositionAndAlias)
            createNewEtherCATDataHolder(xmlIMU.getName() + "_" + name, "IMU", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlIMU xmlIMU)
            createNewEtherCATDataHolder(xmlIMU.getName() + "_" + name, "IMU", xmlIMU.getPosition(), xmlIMU.getAlias(), DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlTemperatureSensor xmlTemperatureSensor)
            createNewEtherCATDataHolder(xmlTemperatureSensor.getName() + "_" + name, "Temp Sensor", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlH4EtherCATJunctionPort xmlH4EtherCATJunctionPort && useParentPositionAndAlias)
            createNewEtherCATDataHolder(xmlH4EtherCATJunctionPort.getName() + "_" + name, "Junction Port", position, alias, DeviceType.BOARD, true);

         else if (daughterDevice instanceof XmlH4EtherCATJunctionPort xmlH4EtherCATJunctionPort)
            createNewEtherCATDataHolder(xmlH4EtherCATJunctionPort.getName() + "_" + name,
                                        "Junction Port",
                                        xmlH4EtherCATJunctionPort.getPosition(),
                                        xmlH4EtherCATJunctionPort.getAlias(),
                                        DeviceType.BOARD,
                                        true);
      }
   }

   /**
    * Create a data holder for an EtherCAT device
    *
    * @param dataHolderName Name of the data holder
    * @param description    Description of the device
    * @param position       EtherCAT position
    * @param alias          EtherCAT alias
    * @param deviceType     Type of EtherCAT device being registered
    */
   protected void createNewEtherCATDataHolder(String dataHolderName, String description, int position, int alias, DeviceType deviceType)
   {
      createNewEtherCATDataHolder(dataHolderName, description, position, alias, deviceType, false);
   }

   /**
    * Create a data holder for an EtherCAT device
    *
    * @param dataHolderName Name of the data holder
    * @param description    Description of the device
    * @param position       EtherCAT position
    * @param alias          EtherCAT alias
    * @param deviceType     Type of EtherCAT device being registered
    * @param childDevice    If true, the device is registered as a child of another device
    */
   protected void createNewEtherCATDataHolder(String dataHolderName, String description, int position, int alias, DeviceType deviceType, boolean childDevice)
   {
      if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX))
         return;

      else if (!doesVariableExist(YoEnum.class, dataHolderName + DeviceStatusHolder.STATE_SUFFIX))
         return;

      if (childDevice)
         deviceDataHolders.add(new UIDeviceStatusHolder(dataHolderName,
                                                        "",
                                                        description,
                                                        sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX)
                                                                                 .getYoVariable(),
                                                        sessionVisualizerControls.newYoEnumProperty(dataHolderName + DeviceStatusHolder.STATE_SUFFIX),
                                                        position,
                                                        alias,
                                                        deviceType));
      else
         deviceDataHolders.add(new UIDeviceStatusHolder(dataHolderName,
                                                        description,
                                                        "",
                                                        sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX)
                                                                                 .getYoVariable(),
                                                        sessionVisualizerControls.newYoEnumProperty(dataHolderName + DeviceStatusHolder.STATE_SUFFIX),
                                                        position,
                                                        alias,
                                                        deviceType));
   }

   protected void createNewElmoTwitterDataHolder(String dataHolderName, String description, int position, int alias, DeviceType deviceType)
   {
      if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX))
         return;

      else if (!doesVariableExist(YoEnum.class, dataHolderName + DeviceStatusHolder.STATE_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.IS_FAULTED_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.UNDER_VOLTAGE_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.OVER_VOLTAGE_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.STO_DISABLED_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.CURRENT_SHORT_SUFFIX))
         return;

      else if (!doesVariableExist(YoBoolean.class, dataHolderName + DeviceStatusHolder.OVER_TEMP_SUFFIX))
         return;

      else if (!doesVariableExist(YoInteger.class, dataHolderName + DeviceStatusHolder.ELMO_ERROR_CODE_SUFFIX))
         return;

      else if (!doesVariableExist(YoDouble.class, dataHolderName + DeviceStatusHolder.INPUT_ENCODER_ERROR_SUFFIX))
         return;

      else if (!doesVariableExist(YoDouble.class, dataHolderName + DeviceStatusHolder.OUTPUT_ENCODER_ERROR_SUFFIX))
         return;

      deviceDataHolders.add(new UIDeviceStatusHolder(dataHolderName,
                                                     description,
                                                     "",
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_RESPONDING_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoEnumProperty(dataHolderName + DeviceStatusHolder.STATE_SUFFIX),
                                                     position,
                                                     alias,
                                                     deviceType,
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.IS_FAULTED_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.UNDER_VOLTAGE_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.OVER_VOLTAGE_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.STO_DISABLED_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.CURRENT_SHORT_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoBooleanProperty(dataHolderName + DeviceStatusHolder.OVER_TEMP_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoIntegerProperty(dataHolderName + DeviceStatusHolder.ELMO_ERROR_CODE_SUFFIX)
                                                                              .getYoVariable(),
                                                     sessionVisualizerControls.newYoDoubleProperty(
                                                           dataHolderName + DeviceStatusHolder.INPUT_ENCODER_ERROR_SUFFIX).getYoVariable(),
                                                     sessionVisualizerControls.newYoDoubleProperty(
                                                           dataHolderName + DeviceStatusHolder.OUTPUT_ENCODER_ERROR_SUFFIX).getYoVariable()));
   }

   /**
    * Checks to make sure the yovariable exists
    *
    * @param type         Type of yovariable being checked
    * @param variableName Name of the yovariable
    * @param <T>          Class type must extend {@code YoVariable}
    * @return True if the yovariable exists
    */
   protected <T extends YoVariable> boolean doesVariableExist(Class<T> type, String variableName)
   {
      return findVariable(type, variableName) != null;
   }

   private <T extends YoVariable> T findVariable(Class<T> type, String variableName)
   {
      int separatorIndex = variableName.lastIndexOf(YoTools.NAMESPACE_SEPERATOR_STRING);

      String namespaceEnding = separatorIndex == -1 ? null : variableName.substring(0, separatorIndex);
      String name = separatorIndex == -1 ? variableName : variableName.substring(separatorIndex + 1);
      return (T) YoSearchTools.findFirstVariable(namespaceEnding, name, type::isInstance, toolkit.getYoManager().getRootRegistry());
   }

   private boolean hasHardwareStatusVariables(List<AbstractXmlDevice> xmlDevices)
   {
      for (AbstractXmlDevice xmlDevice : xmlDevices)
      {
         if (xmlDevice.isPresent() && doesVariableExist(YoBoolean.class, xmlDevice.getName() + DeviceStatusHolder.IS_RESPONDING_SUFFIX))
            return true;
      }
      return false;
   }

   public ArrayList<UIDeviceStatusHolder> getDeviceDataHolders()
   {
      return deviceDataHolders;
   }
}
