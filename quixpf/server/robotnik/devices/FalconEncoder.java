package com._604robotics.robotnik.devices;

import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class FalconEncoder implements IntegratedEncoder {
  private final TalonFXSensorCollection sensors;

  private CalculableRatio ratio;

  private double conversionFactor = 1.0;
  private boolean inverted = false;

  public FalconEncoder(QuixTalonFX talon) {
    sensors = talon.controller.getSensorCollection();
    ratio = null;
  }

  public FalconEncoder(QuixTalonFX talon, CalculableRatio ratio) {
    sensors = talon.controller.getSensorCollection();
    this.ratio = ratio;
    conversionFactor = ratio.calculate(1.0);
  }

  @Override
  public boolean getInverted() {
    return inverted;
  }

  @Override
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  @Override
  public void setdistancePerRotation(double distancePerRotation) {
    if (ratio == null) {
      conversionFactor = distancePerRotation * (1.0 / 2048.0);
    } else {
      conversionFactor = ratio.calculate(distancePerRotation * (1.0 / 2048.0));
    }
  }

  @Override
  public double getPositionConversionFactor() {
    return conversionFactor;
  }

  @Override
  public double getVelocityConversionFactor() {
    return conversionFactor * (1000.0 / 1.0);
  }
  
  @Override
  public void zero() {
    sensors.setIntegratedSensorPosition(0.0, 0);
  }
  
  @Override
  public void zero(double value) {
    sensors.setIntegratedSensorPosition(value / conversionFactor, 0);
  }
  
  @Override
  public double getPosition() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return sensors.getIntegratedSensorPosition() * factor * conversionFactor;
  }
  
  @Override
  public double getVelocity() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return sensors.getIntegratedSensorVelocity() * factor * conversionFactor * (1000.0 / 100.0);
  }
}
