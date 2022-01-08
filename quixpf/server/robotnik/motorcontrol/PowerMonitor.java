package com._604robotics.robotnik.motorcontrol;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;

public class PowerMonitor {
  private static PowerMonitor single_instance = null;

  public final PowerDistributionPanel panel;
  public final Compressor compressor;

  private final NetworkTableInstance network = NetworkTableInstance.getDefault();
  private final NetworkTable table = network.getTable("powermonitor");

  private HashMap<String, MotorController> controllers = new HashMap<>();

  private final static Logger logger = Logger.getGlobal();
  

  public int PDPchannel;

  public PowerMonitor(int PDPchannel, int CompressorID) {
    this.PDPchannel = PDPchannel;

    panel = new PowerDistributionPanel(PDPchannel);
    compressor = new Compressor(CompressorID);

    table.getEntry("Compressor Current").setDouble(compressor.getCompressorCurrent());
    table.getEntry("Compressor State").setBoolean(compressor.enabled());
    table.getEntry("Is Compressed").setBoolean(compressor.getPressureSwitchValue());
  }

  /**
   * @param name
   * @return MotorController
   */
  public MotorController getController(String name) {
    return controllers.getOrDefault(name, null);
  }

  /**
   * @param controller
   * @param name
   */
  public void addController(MotorController controller, String name) {
    controllers.put(name, controller);
    
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Current")
        .setDouble(controller.getOutputCurrent());
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Limit")
        .setDouble(controller.getCurrentLimit());
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Limiting")
        .setBoolean(controller.isCurrentLimiting());
  }

  public void update() {
    for (Map.Entry<String, MotorController> mapElement : controllers.entrySet()) {
      String name = mapElement.getKey();
      MotorController controller = mapElement.getValue();

      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Current")
          .setDouble(controller.getOutputCurrent());
      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Limit")
          .setDouble(controller.getCurrentLimit());
      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Limiting")
          .setBoolean(controller.isCurrentLimiting());
    }


    table.getEntry("Compressor Current").setDouble(compressor.getCompressorCurrent());
    table.getEntry("Compressor State").setBoolean(compressor.enabled());
    table.getEntry("Is Compressed").setBoolean(compressor.getPressureSwitchValue());

    checkFaults();
  }

  /** @param state */
  public void updateCompressor(boolean state) {
    if (state) {
      compressor.start();
    } else {
      compressor.stop();
    }
  }

  @SuppressWarnings("unused")
  private void checkFaults() {
    if (compressor.getCompressorCurrentTooHighFault()) {
      logger.warning("Compressor overcurrent fault!");
    } else if (compressor.getCompressorNotConnectedFault()) {
      logger.warning("Compressor is not connected!");
    } else if (compressor.getCompressorShortedFault()) {
      logger.warning("Compressor was shorted!");
    }
  }

  /**
   * @param PDPchannel
   * @param CompressorID
   * @return PowerMonitor
   */
  public static PowerMonitor getInstance(int PDPchannel, int CompressorID) {
    if (single_instance == null) single_instance = new PowerMonitor(PDPchannel, CompressorID);

    return single_instance;
  }

  public static PowerMonitor getInstance() {
    if (single_instance == null) single_instance = new PowerMonitor(1, 0);

    return single_instance;
  }
}
