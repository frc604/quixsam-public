
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;

import org.junit.jupiter.api.Test;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;
import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
import org.knowm.xchart.style.markers.Marker;
import org.knowm.xchart.style.markers.SeriesMarkers;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class quikobstest {

  @Test
  void graph() {
    ArrayList<Pose2d> robotPoses = new ArrayList<Pose2d>();
    ArrayList<Pose2d> cameraPoses = new ArrayList<Pose2d>();
    ArrayList<Translation2d> targetTranslations = new ArrayList<Translation2d>();

    Transform2d robotToCamera = new Transform2d(new Translation2d(0.1, 0), new Rotation2d());

    var pose1 = new Pose2d(0, 0, new Rotation2d());
    var pose2 = new Pose2d(1, 1, Rotation2d.fromDegrees(45));

    robotPoses.add(pose1);
    robotPoses.add(pose2);

    cameraPoses.add(pose1.transformBy(robotToCamera));
    cameraPoses.add(pose2.transformBy(robotToCamera));

    targetTranslations.add(new Translation2d(3, 3));
    targetTranslations.add(new Translation2d(-2, 3));
    targetTranslations.add(new Translation2d(6, 4));
    targetTranslations.add(new Translation2d(-1, -1));


    XYChart chart =
        new XYChartBuilder()
            .width(800)
            .height(600)
            .title(getClass().getSimpleName())
            .xAxisTitle("X")
            .yAxisTitle("Y")
            .build();

    chart.getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Scatter);
    chart.getStyler().setLegendVisible(false);
    chart.getStyler().setMarkerSize(20);

    for (Pose2d pose : robotPoses) {
      chart.addSeries(pose.toString(), new double[]{pose.getX()}, new double[]{pose.getY()})
        .setMarker(new PoseMarker(pose.getRotation(), Color.RED, Color.BLUE));
    }

    for (Pose2d pose : cameraPoses) {
      chart.addSeries(pose.toString(), new double[]{pose.getX()}, new double[]{pose.getY()})
      .setMarker(new PoseMarker(pose.getRotation(), Color.GREEN, Color.BLUE));
    }

    for (Translation2d translation : targetTranslations) {
      chart.addSeries(translation.toString(), new double[]{translation.getX()}, new double[]{translation.getY()})
      .setMarker(SeriesMarkers.TRIANGLE_UP);
    }

    new SwingWrapper(chart).displayChart();

    try {
      Thread.sleep(2000000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  public class PoseMarker extends Marker {
    private Rotation2d rotation;
    private Color xColor;
    private Color yColor;

    public PoseMarker(Rotation2d rotation, Color xColor, Color yColor) {
      this.rotation = rotation;
      this.xColor = xColor;
      this.yColor = yColor;
    }

    @Override
    public void paint(Graphics2D g, double xOffset, double yOffset, int markerSize) {
      g.setStroke(new BasicStroke(5, BasicStroke.CAP_ROUND, BasicStroke.CAP_BUTT));
      // Note y-direction for markers is inverted.
      g.setColor(yColor);
      g.drawLine((int) xOffset, (int) yOffset, (int) (xOffset + markerSize * rotation.getSin()), (int) (yOffset + markerSize * rotation.getCos()));
      g.setColor(xColor);
      g.drawLine((int) xOffset, (int) yOffset, (int) (xOffset + markerSize * rotation.getCos()), (int) (yOffset - markerSize * rotation.getSin()));
    }
  }
}