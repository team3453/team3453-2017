package org.usfirst.frc.team3453.lib.util;

public class Command {
	
	String command;
	double parm1;
	double parm2;
	
	public Command (String command, double parm1, double parm2) {
		this.command = command;
		this.parm1 = parm1;
		this.parm2 = parm2;
	}
	
	public String getCommand () {
		return command;
	}
	public double getParm1() {
		return parm1;
	}
	public double getParm2() {
		return parm2;
	}
}