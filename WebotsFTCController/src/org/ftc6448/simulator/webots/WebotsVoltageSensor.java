package org.ftc6448.simulator.webots;

import com.qualcomm.robotcore.hardware.VoltageSensor;

public class WebotsVoltageSensor implements VoltageSensor {

	@Override
	public Manufacturer getManufacturer() {
		return Manufacturer.Lynx;
	}

	@Override
	public String getDeviceName() {
		return "Webots Voltage Sensor";
	}

	@Override
	public String getConnectionInfo() {
		return null;
	}

	@Override
	public int getVersion() {
		return 1;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
		
	}

	@Override
	public void close() {
		
	}

	@Override
	public double getVoltage() {
		return 13;
	}

}
