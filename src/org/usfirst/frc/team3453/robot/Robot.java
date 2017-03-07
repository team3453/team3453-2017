package org.usfirst.frc.team3453.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import org.usfirst.frc.team3453.lib.util.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	// true for Competition bot, false for Practice bot
	private boolean isCompetitionBot = false;
	
	private boolean pressureGood = false;
	
	private boolean airOn = false;
	private int currentCount = 0;
	
	private navxmxp_data_monitor ahrs;
	
	SpeedController _frontLeftMotor;
	SpeedController _rearLeftMotor;
	SpeedController _frontRightMotor;
	SpeedController _rearRightMotor;

	SpeedController _climber;
	SpeedController _fuelIntake;
	SpeedController _fuelShooter;

	RobotDrive _drive;

	Joystick _gamepad = new Joystick(0);
	Joystick _joy = new Joystick(1);
	
	//RobotDrive myRobot = new RobotDrive(0, 1);
	Timer timer = new Timer();
	int count = 0;
	
	Compressor c = new Compressor(0);
	DoubleSolenoid sol_01 = new DoubleSolenoid(0, 0, 1);
	DoubleSolenoid sol_23 = new DoubleSolenoid(0, 2, 3);
	
	SendableChooser chooser;
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	
	CameraServer server; //CameraServer.getInstance();
	UsbCamera cam0;      //new UsbCamera("cam0",0);
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

		
	
	@Override
	public void robotInit() {
		
		if (isCompetitionBot) {
		    // Competition use - Talon SRXs

			_frontLeftMotor  = new CANTalon(1);     //device IDs here (1 of 2)
			_rearLeftMotor   = new CANTalon(2);
			_frontRightMotor = new CANTalon(3);
			_rearRightMotor  = new CANTalon(4);
			
			// set all Talon SRX drive motors to Coast from software
			((CANTalon) _frontLeftMotor).enableBrakeMode(false);
			((CANTalon) _rearLeftMotor).enableBrakeMode(false);
			((CANTalon) _frontRightMotor).enableBrakeMode(false);
			((CANTalon) _rearRightMotor).enableBrakeMode(false);
			
		    // Competition use
			_climber = new CANTalon(5);
			_fuelIntake = new CANTalon(6);
			_fuelShooter = new CANTalon(7);
			
		} else {
			// Practice Robot use - Sparks

			_frontLeftMotor  = new Spark(0); 		//device IDs here (1 of 2)
			_rearLeftMotor   = new Spark(1);
			_frontRightMotor = new Spark(2);
			_rearRightMotor  = new Spark(3);
			
			_climber = new Victor(5);
			_fuelIntake = new Victor(6);
			_fuelShooter = new Victor(7);
		}
		
		//_drive = new RobotDrive(_frontRightMotor, _rearRightMotor, _frontLeftMotor, _rearLeftMotor);
		_drive = new RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
		
		// use if drive motor/gearbox runs backwards.
		_drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft,true);
		_drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		_drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		_drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
				
		c.setClosedLoopControl(true);
		pressureGood = false;
		driveNeutral();
		//sol_11.set(false);
		
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
		
		//startAutomaticCapture(cam0, 0);//string name(the one you call it by, device id number)
		
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(320, 240);
			camera.setBrightness(20);
			camera.setExposureAuto();
			camera.setFPS(25);
			camera.setWhiteBalanceAuto();
			
            
            /*
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
            */
		}).start();
        
		ahrs = new navxmxp_data_monitor();
		
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		ahrs.zeroYaw();
		count = 0;
		
		c.setClosedLoopControl(true);
		pressureGood = false;
		driveLo();
		_frontLeftMotor.set(0);
		_frontRightMotor.set(0);
		_rearLeftMotor.set(0);
		_rearRightMotor.set(0);
		
		_climber.set(0);
		_fuelIntake.set(0);
		_fuelShooter.set(0);		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		ahrs.printStats();
		count++;
		
		// Drive for 2 seconds
		/*
		if (timer.get() < 2.0) {
			_drive.drive(-0.5, 0.0); // drive forwards half speed
		} else {
			_drive.drive(0.0, 0.0); // stop robot
		}  */
		
		String autoSelected = (String) chooser.getSelected();
		//String autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);

		switch(autoSelected) {
		case customAuto:
			_drive.setSafetyEnabled(false);
			if (count < 100) {
				_drive.drive(-0.5, 1.0); // spin at half speed
			} else {
				//Timer.delay(2.0);		 //    for 2 seconds
				_drive.drive(0.0, 0.0);	 // stop robot
			}
			break;
		case defaultAuto:
		default:
			_drive.setSafetyEnabled(false);
			if (count < 100) { // spin for 2 seconds
				_drive.drive(-0.5, 0.0); // drive forwards half speed
			} else {
				//Timer.delay(2.0);		 //    for 2 seconds
				_drive.drive(0.0, 0.0);	 // stop robot
			}
			break;
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		ahrs.zeroYaw();
		c.setClosedLoopControl(true);
		pressureGood = false;
		driveLo();
		_drive.drive(0.0, 0.0); // stop drive if still running
		_frontLeftMotor.set(0);
		_frontRightMotor.set(0);
		_rearLeftMotor.set(0);
		_rearRightMotor.set(0);
		
		_climber.set(0);
		_fuelIntake.set(0);
		_fuelShooter.set(0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		ahrs.printStats();
		
		if(c.getPressureSwitchValue()){ //pressure switch good then set true else be false
			pressureGood = false;
		} else {
			pressureGood = true;
		}
		
		// driver input
		double forward = _gamepad.getRawAxis(1); // logitech gamepad left Y, positive is forward
    	double turn = _gamepad.getRawAxis(4); //logitech gamepad right X, positive means turn right
    
    	if (_gamepad.getRawButton(6)){ // right shoulder
    		driveHi();
    	} else {
    		driveLo();
    	}
    	
    	
    	    	
    	if (_gamepad.getRawButton(1)){ // button X
    		// practice buttons
			_rearRightMotor.set(0.1);
			_frontRightMotor.set(0.1);
	    } else {
	    	_drive.arcadeDrive(forward, turn);
	    }
	

    	// operator input
    	double input = _joy.getY();  // push forward is negative values
    	boolean trigger = _joy.getRawButton(1); // trigger
    	boolean shooter = _joy.getRawButton(3); // top
   	
    	double climberInput = Math.abs(input);
    	if (_joy.getRawButton(2) && (climberInput > 0.15) ){  // joystick button 2
    		
    		// rescale and limit max climber motor output to 0.7
    		climberInput = climberInput * 0.7;  
 
    		_climber.set(-climberInput); // input will turn motor clockwise
    	} else {
    		_climber.set(0);
    	}
    	
    	if (trigger) {
    		_fuelIntake.set(-0.95);
    	} else {
    		_fuelIntake.set(0);
    	}
    	
    	if(shooter) {
    		_fuelShooter.set(-0.75);//placeholder for test
    	} else {
    		_fuelShooter.set(0);
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
 

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	@Override
	public void disabledInit() {
		ahrs.zeroYaw();
		c.setClosedLoopControl(false);
		pressureGood = false;
		driveLo();
		_frontLeftMotor.set(0);
		_frontRightMotor.set(0);
		_rearLeftMotor.set(0);
		_rearRightMotor.set(0);
		
		_climber.set(0);
		_fuelIntake.set(0);
		_fuelShooter.set(0);		
	}
	
	@Override
	public void disabledPeriodic() {
		
	}
	
	@Override
	public void robotPeriodic() {
		// exactly 20ms tasks in here
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
