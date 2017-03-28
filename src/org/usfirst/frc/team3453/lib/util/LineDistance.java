package org.usfirst.frc.team3453.lib.util;

public class LineDistance {
	
	private double startX;
	private double startY;
	
	private double X;
	private double Y;
	
	public LineDistance (double X, double Y) {
		this.startX = X;
		this.startY = Y;
	}
	
	public double getDistance (double X, double Y) {
		double d = Math.sqrt(Math.pow((startX - X), 2.0) + Math.pow((startY - Y), 2.0));
		
		return d;
	}

}
