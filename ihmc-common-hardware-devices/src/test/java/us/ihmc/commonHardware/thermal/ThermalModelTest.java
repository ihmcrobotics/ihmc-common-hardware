package us.ihmc.commonHardware.thermal;

import org.junit.jupiter.api.Test;
import org.knowm.xchart.BitmapEncoder;
import org.knowm.xchart.BitmapEncoder.BitmapFormat;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries;
import org.knowm.xchart.style.Styler;
import org.knowm.xchart.style.Styler.LegendPosition;
import org.knowm.xchart.style.markers.SeriesMarkers;
import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlActuatorParameterLoader;
import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlMotorParameters;
import us.ihmc.hardwareXMLToolkit.devices.parameters.XmlMotorThermalParameters;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.File;
import java.io.IOException;

public class ThermalModelTest
{
   private static final double DEFAULT_TEMPERATURE = 25.0;
   private static final String PLOT_PATH = "build/reports/thermalModelTest/";

   // Wide enough dashes/width to stay legible at any plot resolution, unlike XChart's default hairline SeriesLines.DASH_DASH
   private static final BasicStroke SOLID_LINE = new BasicStroke(2.5f);
   private static final BasicStroke DASHED_LINE = new BasicStroke(2.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_ROUND, 1.0f, new float[] {14f, 10f}, 0f);

   /**
    * Run thermal model sim for TQ ILM 115x25 at 25%, 50%, 75%, and 100% peak current
    */
   @Test
   public void testThermalModels115() throws IOException
   {
      XmlMotorParameters tq115MotorParams = XmlActuatorParameterLoader.getMotorParametersFromMotorName("tq_ILM115x25");
      testThermalModelAtDifferentCurrents(tq115MotorParams);
   }

   /**
    * Run thermal model sim for TQ ILM 85x26 at 25%, 50%, 75%, and 100% peak current
    */
   @Test
   public void testThermalModels85() throws IOException
   {
      XmlMotorParameters tq115MotorParams = XmlActuatorParameterLoader.getMotorParametersFromMotorName("tq_ILM85x26");
      testThermalModelAtDifferentCurrents(tq115MotorParams);
   }

   /**
    * Run thermal model sim for Kollmorgen TBM2G 07626C at 25%, 50%, 75%, and 100% peak current
    */
   @Test
   public void testThermalModels76() throws IOException
   {
      XmlMotorParameters tq115MotorParams = XmlActuatorParameterLoader.getMotorParametersFromMotorName("kollmorgen_TBM2G_07626C");
      testThermalModelAtDifferentCurrents(tq115MotorParams);
   }

   /**
    * Run thermal model sim for Kollmorgen TBM2G 06826C at 25%, 50%, 75%, and 100% peak current
    */
   @Test
   public void testThermalModels68() throws IOException
   {
      XmlMotorParameters tq115MotorParams = XmlActuatorParameterLoader.getMotorParametersFromMotorName("kollmorgen_TBM2G_06826C");
      testThermalModelAtDifferentCurrents(tq115MotorParams);
   }

   private void testThermalModelAtDifferentCurrents(XmlMotorParameters xmlMotorParams) throws IOException
   {
      // Get important fields from XML
      XmlMotorThermalParameters xmlMotorThermalParameters = xmlMotorParams.getMotorThermalParameters();
      String manufacturer = xmlMotorParams.getManufacturer();
      String model = xmlMotorParams.getModel();

      // Construct model for each current level we will test
      MotorThermalModel motorThermalModel25Percent = new MotorThermalModel(new MotorThermalParameters(xmlMotorThermalParameters), DEFAULT_TEMPERATURE, DEFAULT_TEMPERATURE);
      MotorThermalModel motorThermalModel50Percent = new MotorThermalModel(new MotorThermalParameters(xmlMotorThermalParameters), DEFAULT_TEMPERATURE, DEFAULT_TEMPERATURE);
      MotorThermalModel motorThermalModel75Percent = new MotorThermalModel(new MotorThermalParameters(xmlMotorThermalParameters), DEFAULT_TEMPERATURE, DEFAULT_TEMPERATURE);
      MotorThermalModel motorThermalModel100Percent = new MotorThermalModel(new MotorThermalParameters(xmlMotorThermalParameters), DEFAULT_TEMPERATURE, DEFAULT_TEMPERATURE);

      // Params for our sim
      double maxCurrent = xmlMotorParams.getMaxCurrentPeak();
      double dt = 0.001; // seconds
      double simDuration = 180.0; //seconds

      // Some arrays
      MotorThermalModel[] models = {motorThermalModel25Percent, motorThermalModel50Percent, motorThermalModel75Percent, motorThermalModel100Percent};
      Color[] modelColors = {Color.RED, Color.BLUE, new Color(0, 153, 0), Color.ORANGE};
      double[] appliedCurrents = {0.25 * maxCurrent, 0.5 * maxCurrent, 0.75 * maxCurrent, maxCurrent};
      String[] modelLabels = {"25% peak current (" + appliedCurrents[0] + " A)",
                              "50% peak current (" + appliedCurrents[1] + " A)",
                              "75% peak current (" + appliedCurrents[2] + " A)",
                              "100% peak current (" + appliedCurrents[3] + " A)"};

      // The series we will plot
      int totalSteps = (int) Math.round(simDuration / dt);
      double[] time = new double[totalSteps + 1];
      double[][] windingTemperature = new double[models.length][time.length];
      double[][] housingTemperature = new double[models.length][time.length];

      // Calculate the temps at t=0
      for (int i = 0; i < models.length; i++)
      {
         windingTemperature[i][0] = models[i].getWindingTemperature();
         housingTemperature[i][0] = models[i].getHousingTemperature();
      }

      // Calculate the temps for the remainder of the simulation
      for (int step = 1; step <= totalSteps; step++)
      {
         time[step] = step * dt;
         for (int i = 0; i < models.length; i++)
         {
            models[i].update(appliedCurrents[i], dt);
            windingTemperature[i][step] = models[i].getWindingTemperature();
            housingTemperature[i][step] = models[i].getHousingTemperature();
         }
      }

      // Build the chart
      XYChart chart = new XYChartBuilder().width(1920)
                                          .height(1080)
                                          .title(manufacturer + " " + model + " Winding & Housing Temperature vs Time")
                                          .xAxisTitle("Time (s)")
                                          .yAxisTitle("Temperature (°C)")
                                          .build();
      chart.getStyler().setLegendPosition(LegendPosition.InsideNW);

      // Params for downsampling
      int windingPlotPoints = 1800;
      int housingPlotPoints = 60;

      // Add the data to our chart
      for (int i = 0; i < models.length; i++)
      {
         addSeries(chart,
                   modelLabels[i] + " - winding",
                   downsample(time, windingPlotPoints),
                   downsample(windingTemperature[i], windingPlotPoints),
                   modelColors[i],
                   SOLID_LINE);
         addSeries(chart,
                   modelLabels[i] + " - housing",
                   downsample(time, housingPlotPoints),
                   downsample(housingTemperature[i], housingPlotPoints),
                   modelColors[i],
                   DASHED_LINE);
      }

      // Export the plots as pngs
      String plotName = "thermalModel_" + manufacturer + "_" + model;
      new File(PLOT_PATH + plotName).getParentFile().mkdirs();
      BitmapEncoder.saveBitmap(chart, PLOT_PATH + plotName, BitmapFormat.PNG);
   }

   // Adds an arbitrary series to a chart
   private static void addSeries(XYChart chart, String name, double[] time, double[] temperature, Color color, BasicStroke lineStyle)
   {
      XYSeries series = chart.addSeries(name, time, temperature);
      series.setMarker(SeriesMarkers.NONE);
      series.setLineColor(color);
      series.setLineStyle(lineStyle);
   }

   // Downsamples raw data to a data series with fewer points (number of points is targetCount)
   private static double[] downsample(double[] data, int targetCount)
   {
      double[] result = new double[targetCount];
      for (int i = 0; i < targetCount; i++)
         result[i] = data[(int) Math.round(i * (data.length - 1) / (double) (targetCount - 1))];
      return result;
   }
}
