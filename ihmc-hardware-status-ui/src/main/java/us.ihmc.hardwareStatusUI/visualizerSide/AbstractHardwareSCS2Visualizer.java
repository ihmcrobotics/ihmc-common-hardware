package us.ihmc.hardwareStatusUI.visualizerSide;

import javafx.scene.Node;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import java.util.ArrayList;

public class AbstractHardwareSCS2Visualizer
{
   protected final SessionVisualizerControls sessionVisualizerControls;
   protected final SessionVisualizerToolkit toolkit;
   private final ArrayList<Node> startupSequencePaneNodes = new ArrayList<>();
   private final ArrayList<Node> shutdownSequencePaneNodes = new ArrayList<>();
   private final ArrayList<Node> debuggingPaneNodes = new ArrayList<>();

   public AbstractHardwareSCS2Visualizer(SessionVisualizer sessionVisualizer)
   {
      sessionVisualizerControls = sessionVisualizer.getSessionVisualizerControls();
      toolkit = sessionVisualizer.getToolkit();
   }

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

   private void createPane(SessionVisualizerControls sessionVisualizerControls, String paneDescription, ArrayList<Node> children)
   {
      Pane pane = new Pane();

      for (int i = 0; i < children.size(); i ++)
      {
         Node child = children.get(i);
         pane.getChildren().add(child);
         child.relocate(5, (i+1)*5 + i*35);
      }

      sessionVisualizerControls.addCustomGUIPane(paneDescription, pane);
   }
}
