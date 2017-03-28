package org.usfirst.frc.team3453.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sonar {
	
	private AnalogInput input;
	
	public Sonar () {
		input = new AnalogInput(0);
	}
	
	public double getDistance () {
		double v = getVoltage();
		double d = v/(9.8*0.001); // 9.8 millivolts = 1 inch
		SmartDashboard.putNumber(   "Sonar Distance:  ", d);  
		SmartDashboard.putNumber(   "Sonar Dist.:  ", d);
		return d;
	}
	
	public double getVoltage() {
		double v = input.getVoltage();
		SmartDashboard.putNumber(   "Sonar Voltage:  ", v);  
		return v;
	}

}
