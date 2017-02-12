package org.usfirst.frc.team3453.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public boolean pressureGood = false;
	
	public boolean airOn = false;
	public int currentCount = 0;
	
	/* talons for arcade drive */
	CANTalon _frontLeftMotor = new CANTalon(1); 		/* device IDs here (1 of 2) */
	CANTalon _rearLeftMotor = new CANTalon(2);
	CANTalon _frontRightMotor = new CANTalon(3);
	CANTalon _rearRightMotor = new CANTalon(4);
	
	CANTalon _placeHolder1 = new CANTalon(5);
	CANTalon _placeHolder2 = new CANTalon(6);
	CANTalon _placeHolder3 = new CANTalon(7);
	
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
		sol_01.set(DoubleSolenoid.Value.kOff);
		sol_23.set(DoubleSolenoid.Value.kOff);
		//sol_11.set(false);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (timer.get() < 2.0) {
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		c.setClosedLoopControl(true);
		pressureGood = false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		if(c.getPressureSwitchValue()){ //pressure switch good then set true else be false
			pressureGood = false;
		} else {
			pressureGood = true;
		}
		
		double forward = _gamepad.getRawAxis(1); // logitech gamepad left Y, positive is forward
    	double turn = _gamepad.getRawAxis(4); //logitech gamepad right X, positive means turn right
    
    	_drive.arcadeDrive(forward, -turn);

    	if(_joy.getRawButton(3)){
    		_placeHolder1.set(0.1);
    	}
    	
		/*
   	if (pressureGood) {
    		currentCount++;
    		if (currentCount > 100) {
    			currentCount = 0;
    			if (airOn) {
    				sol_01.set(DoubleSolenoid.Value.kOff);
    				sol_23.set(DoubleSolenoid.Value.kOff);
    				//sol_11.set(false);
    			} else {
    				airOn = true;    				
    				sol_01.set(DoubleSolenoid.Value.kForward);
    				sol_23.set(DoubleSolenoid.Value.kForward);
    				//sol_11.set(true);
    			}
    		}
    	}
 * */
 
    	if (_gamepad.getRawButton(1)){
    			_rearRightMotor.set(0.1);
    			_frontRightMotor.set(0.1);
    	}
    	

    	if (_gamepad.getRawButton(6)){
    		sol_01.set(DoubleSolenoid.Value.kReverse);
    		sol_23.set(DoubleSolenoid.Value.kReverse);
    	} else {
    		sol_01.set(DoubleSolenoid.Value.kForward);
    		sol_23.set(DoubleSolenoid.Value.kForward);
    	}

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
