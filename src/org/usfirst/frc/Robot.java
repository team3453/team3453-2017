package org.usfirst.frc.team3453.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import com.ctre.CANTalon;


public class Robot extends SampleRobot {
	
	public boolean pressureGood = false;
	
	public boolean airOn = false;
	public int currentCount = 0;
	
	/* talons for arcade drive */
	CANTalon _frontLeftMotor = new CANTalon(1); 		/* device IDs here (1 of 2) */
	CANTalon _rearLeftMotor = new CANTalon(2);
	CANTalon _frontRightMotor = new CANTalon(3);
	CANTalon _rearRightMotor = new CANTalon(4);
	
	CANTalon _climber = new CANTalon(5);
	CANTalon _ballIntake = new CANTalon(6);
	CANTalon _ballShooter = new CANTalon(7);
	
	RobotDrive _drive = new RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);

	Joystick _gamepad = new Joystick(0);
	Joystick _joy = new Joystick(1);
	
	RobotDrive myRobot = new RobotDrive(0, 1);
	Timer timer = new Timer();
	
	Compressor c = new Compressor(9);
	DoubleSolenoid sol_01 = new DoubleSolenoid(0, 0, 1);
	DoubleSolenoid sol_23 = new DoubleSolenoid(0, 2, 3);
	//Solenoid sol_11 = new Solenoid(0);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		c.setClosedLoopControl(true);
		pressureGood = false;
		driveNeutral();
		//sol_11.set(false);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomous() {
		timer.reset();
		timer.start();
		
		/**
		 * This autonomous (along with the chooser code above) shows how to select
		 * between different autonomous modes using the dashboard. The sendable
		 * chooser code works with the Java SmartDashboard. If you prefer the
		 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
		 * getString line to get the auto name from the text box below the Gyro
		 *
		 * You can add additional auto modes by adding additional comparisons to the
		 * if-else structure below with additional strings. If using the
		 * SendableChooser make sure to add them to the chooser code above as well.
		 */
		
		String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		/**
		 * Purely from the preset code when using Sample Robot 
		 */
		
		switch (autoSelected) {
		case customAuto:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 1.0); // spin at half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		case defaultAuto:
		default:
			myRobot.setSafetyEnabled(false);
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
			Timer.delay(2.0); // for 2 seconds
			myRobot.drive(0.0, 0.0); // stop robot
			break;
		}
		
	}

	public void operatorControl(){
		
		c.setClosedLoopControl(true);
		pressureGood = false;
		driveLo();
		
		myRobot.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			if(c.getPressureSwitchValue()){ //pressure switch good then set true else be false
				pressureGood = false;
			} else {
				pressureGood = true;
			}
			
			double forward = _gamepad.getRawAxis(1); // logitech gamepad left Y, positive is forward
	    	double turn = _gamepad.getRawAxis(4); //logitech gamepad right X, positive means turn right
	    
	    	_drive.arcadeDrive(forward, -turn);

	    	if(_joy.getRawButton(10)){
	    		_climber.set(0.5);
	    	}
			
		if(_joy.getRawButton(1)){
			_ballIntake.set(0.5);
		}
		
		if(_joy.getRawButton(2)){//or 3 depending on driver preference
			_ballShooter.set(0.5);
		}
	    	
			/*
	   	if (pressureGood) {
	    		currentCount++;
	    		if (currentCount > 100) {
	    			currentCount = 0;
	    			if (airOn) {
	    				driveHi();
	    				//sol_11.set(false);
	    			} else {
	    				airOn = true;    				
	    				driveLo();
	    				//sol_11.set(true);
	    			}
	    		}
	    	}
	 * */
	 
	    	if (_gamepad.getRawButton(1)){ // button X
	    			_rearRightMotor.set(0.1);
	    			_frontRightMotor.set(0.1);
	    	}
	    	

	    	if (_gamepad.getRawButton(6)){ // right shoulder
	    		driveHi();
	    	} else {
	    		driveLo();
	    	}
		}
		
	}

	@Override
	public void test() {
		LiveWindow.run();
	}
	
	public void driveLo() {
		sol_01.set(DoubleSolenoid.Value.kForward);
		sol_23.set(DoubleSolenoid.Value.kForward);
	}
	
	public void driveHi() {
		sol_01.set(DoubleSolenoid.Value.kReverse);
		sol_23.set(DoubleSolenoid.Value.kReverse);		
	}
	
	public void driveNeutral() {
		sol_01.set(DoubleSolenoid.Value.kOff);
		sol_23.set(DoubleSolenoid.Value.kOff);
	}
}
