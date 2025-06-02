package us.ihmc.hardwareStatusUI.visualizerSide;

import javafx.collections.ObservableList;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.Pane;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;

import java.util.ArrayList;

public class AbstractHardwareSCS2Visualizer
{
   protected final SessionVisualizerControls sessionVisualizerControls;
   protected final SessionVisualizerToolkit toolkit;

   public AbstractHardwareSCS2Visualizer(SessionVisualizer sessionVisualizer)
   {
      sessionVisualizerControls = sessionVisualizer.getSessionVisualizerControls();
      toolkit = sessionVisualizer.getToolkit();
   }

   protected void addHardwareStatusUI(AbstractUIHardwareStatusManager hardwareStatusUIDataManager)
   {
      toolkit.getWindowManager().queueVisualizationController(new HardwareStatusUI(sessionVisualizerControls, hardwareStatusUIDataManager));
   }

   protected void createStartupSequencePane(ArrayList<Node> children)
   {
      createPane(sessionVisualizerControls, "Startup Sequence", children);
   }

   protected void createShutdownSequencePane(ArrayList<Node> children)
   {
      createPane(sessionVisualizerControls, "Shutdown Sequence", children);
   }

   private void createPane(SessionVisualizerControls sessionVisualizerControls, String paneDescription, ArrayList<Node> children)
   {
      Pane pane = new Pane();

      for (int i = 0; i < children.size(); i ++)
      {
         Node child = children.get(i);
         pane.getChildren().add(child);
         child.relocate(5, (i+1) * 5);
      }

      sessionVisualizerControls.addCustomGUIPane(paneDescription, pane);
   }
}
