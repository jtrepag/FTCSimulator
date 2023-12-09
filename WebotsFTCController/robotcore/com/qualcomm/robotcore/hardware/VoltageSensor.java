package com.qualcomm.robotcore.hardware;
public interface VoltageSensor extends HardwareDevice {

  /**
   * Get the current voltage
   * @return voltage
   */
  double getVoltage();
}
