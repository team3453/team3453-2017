package org.usfirst.frc.team3453.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Spark;
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
	
	public boolean pressureGood = false;
	
	public boolean airOn = false;
	public int currentCount = 0;
	
	private AHRS ahrs;
	
	/* talons for arcade drive */
/*
    // Competition use
	SpeedController _frontLeftMotor = new CANTalon(1); 		//device IDs here (1 of 2)
	SpeedController _rearLeftMotor = new CANTalon(2);
	SpeedController _frontRightMotor = new CANTalon(3);
	SpeedController _rearRightMotor = new CANTalon(4);
*/
	// Practice Robot use
	SpeedController _frontLeftMotor = new Spark(0); 		//device IDs here (1 of 2)
	SpeedController _rearLeftMotor = new Spark(1);
	SpeedController _frontRightMotor = new Spark(2);
	SpeedController _rearRightMotor = new Spark(3);
/*
    // Competition use
	CANTalon _climber = new CANTalon(5);
	CANTalon _fuelIntake = new CANTalon(6);
	CANTalon _fuelShooter = new CANTalon(7);
*/
/********************************************
 *  Replace with code below for practice bot
 ********************************************/	

	Spark _climber = new Spark(5);
	Spark _fuelIntake = new Spark(6);
	Spark _fuelShooter = new Spark (7);

	
	//RobotDrive _drive = new RobotDrive(_frontRightMotor, _rearRightMotor, _frontLeftMotor, _rearLeftMotor);
	RobotDrive _drive = new RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);

	Joystick _gamepad = new Joystick(0);
	Joystick _joy = new Joystick(1);
	
	//RobotDrive myRobot = new RobotDrive(0, 1);
	Timer timer = new Timer();
	
	Compressor c = new Compressor(0);
	DoubleSolenoid sol_01 = new DoubleSolenoid(0, 0, 1);
	DoubleSolenoid sol_23 = new DoubleSolenoid(0, 2, 3);
	
	SendableChooser chooser;
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	
	CameraServer server = CameraServer.getInstance();
	UsbCamera cam0 = new UsbCamera("cam0",0);
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */

		
	
	@Override
	public void robotInit() {
		
		// use if drive motor/gearbox runs backwards.
		//_drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft,true);
		//_drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		//_drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		//_drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
		
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
			camera.setResolution(640, 480);
            
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
        
		try {
	          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
	          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
	          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
	          ahrs = new AHRS(SPI.Port.kMXP); 
	          
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		
		// put Gyro Initialization in here
		
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
			_drive.drive(-0.5, 1.0); // spin at half speed
			Timer.delay(2.0);		 //    for 2 seconds
			_drive.drive(0.0, 0.0);	 // stop robot
			break;
		case defaultAuto:
		default:
			_drive.setSafetyEnabled(false);
			_drive.drive(-0.5, 0.0); // drive forwards half speed
			Timer.delay(2.0);		 //    for 2 seconds
			_drive.drive(0.0, 0.0);	 // stop robot
			break;
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
		driveLo();
		_frontLeftMotor.set(0);
		_frontRightMotor.set(0);
		_rearLeftMotor.set(0);
		_rearRightMotor.set(0);
	
	/*	
		_climber.set(0);
		_fuelIntake.set(0);
		_fuelShooter.set(0);
	*/
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
	    	_drive.arcadeDrive(forward, -turn);
	    }
	

    	// operator input
    	double input = _joy.getY();  // push forward is negative values
    	boolean trigger = _joy.getRawButton(1); // trigger
    	boolean shooter = _joy.getRawButton(3); // top
   	
    	double climberInput = Math.abs(input);
    	if (_joy.getRawButton(2) && (climberInput > 0.15) ){  // joystick button 3
    		
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
