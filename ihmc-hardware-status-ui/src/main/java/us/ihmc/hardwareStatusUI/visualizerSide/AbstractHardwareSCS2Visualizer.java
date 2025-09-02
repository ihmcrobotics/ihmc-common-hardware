package us.ihmc.hardwareStatusUI.visualizerSide;

import javafx.scene.Node;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;

import java.util.ArrayList;

/**
 * Abstract class to standardize the hardware visualizer for low level hardware control.
 * It includes lists for the startup, shutdown, and debugging panes to add the necessary buttons.
 * It also includes the ability to add a hardware status UI for easier debuggning
 */
public abstract class AbstractHardwareSCS2Visualizer
{
   protected final SessionVisualizerControls sessionVisualizerControls;
   protected final SessionVisualizerToolkit toolkit;
   protected final ArrayList<Node> startupSequencePaneNodes = new ArrayList<>();
   protected final ArrayList<Node> shutdownSequencePaneNodes = new ArrayList<>();
   protected final ArrayList<Node> debuggingPaneNodes = new ArrayList<>();

   /**
    * Creates the hardware visualizer
    *
    * @param sessionVisualizer Session to be connected to
    */
   public AbstractHardwareSCS2Visualizer(SessionVisualizer sessionVisualizer)
   {
      sessionVisualizerControls = sessionVisualizer.getSessionVisualizerControls();
      toolkit = sessionVisualizer.getToolkit();
   }

   /**
    * Add a {@code HardwareStatusUI} for better debugging of sensors
    *
    * @param hardwareStatusUIDataManager Data manager for the UI
    * @param createLaunchUIButtonPane    If true, create a button to launch the UI manually
    */
   protected void addHardwareStatusUI(AbstractUIHardwareStatusManager hardwareStatusUIDataManager, boolean createLaunchUIButtonPane)
   {
      HardwareStatusUI hardwareStatusUI = new HardwareStatusUI(sessionVisualizerControls, hardwareStatusUIDataManager, createLaunchUIButtonPane);
      if (!createLaunchUIButtonPane)
         addDebuggingPaneNode(hardwareStatusUI.getLaunchUIButton());
      toolkit.getWindowManager().queueVisualizationController(hardwareStatusUI);
   }

   protected void addStartupSequencePaneNode(Node node)
   {
      startupSequencePaneNodes.add(node);
   }

   protected void addShutdownSequencePaneNode(Node node)
   {
      shutdownSequencePaneNodes.add(node);
   }

   protected void addDebuggingPaneNode(Node node)
   {
      debuggingPaneNodes.add(node);
   }

   /**
    * Create a separate pane to hold buttons for easier control of specific variables
    *
    * @param sessionVisualizerControls The controls where the pane will be added
    * @param paneDescription           The name of the pane
    * @param children                  The nodes (buttons) that will be in the pane
    */
   protected void createPane(SessionVisualizerControls sessionVisualizerControls, String paneDescription, ArrayList<Node> children)
   {
      Pane pane = new Pane();

      for (int i = 0; i < children.size(); i++)
      {
         Node child = children.get(i);
         pane.getChildren().add(child);
         child.relocate(5, (i + 1) * 5 + i * 35);
      }

      sessionVisualizerControls.addCustomGUIPane(paneDescription, pane);
   }

   protected void createStartupSequencePane()
   {
      createPane(sessionVisualizerControls, "Startup Sequence", startupSequencePaneNodes);
   }

   protected void createShutdownSequencePane()
   {
      createPane(sessionVisualizerControls, "Shutdown Sequence", shutdownSequencePaneNodes);
   }

   protected void createDebuggingPane()
   {
      createPane(sessionVisualizerControls, "Hardware Status/Debugging", debuggingPaneNodes);
   }
}
