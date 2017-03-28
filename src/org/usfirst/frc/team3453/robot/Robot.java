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
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

import java.util.ArrayList;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	// true for Competition bot, false for Practice bot
	private boolean isCompetitionBot = false;
	
	private boolean pressureGood = false;
	
	private boolean airOn = false;
	private int currentCount = 0;
	private int autonomousMasterCounter = 0;
	
	private navxmxp_data_monitor ahrs;
	
	private boolean haveCAN = false;
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
	
	SendableChooser<String> autoChooser;
	final String defaultAutoFwd = "DefaultFwd";
	final String autoAllianceGearHang = "AllianceGearHang";
	final String customAutoBack = "Back";
	final String customAutoShooter = "Shooter";
	final String customAutoAllianceWag = "Alliance Wag";
	final String visionChase = "Vision Chase";
	
	String autoSelected;
	
	SendableChooser<String> allianceChooser;
	final String defaultAlliance = "red";
	final String blueAlliance = "blue";
	String allianceSelected;
	
	CameraServer server; //CameraServer.getInstance();
	UsbCamera camera;      //new UsbCamera("cam0",0);
	
	
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private VisionThread visionThread;
	private double centerX = 0.0;
	private double turn = 0.0;
	
	private final Object imgLock = new Object();
	
	PIDController turnController;
	double rotateToAngleRate;
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	  
	/* This tuning parameter indicates how close to "on target" the    */
	/* PID Controller will attempt to get.                             */

	static final double kToleranceDegrees = 2.0f;
	
	boolean rotateToAngle = false;
	
	Sonar sonar;
	
	// Custom flags for autonomous modes
	LineDistance dist;
	boolean stage1start = false;
	boolean stage2start = false;
	boolean stage3start = false;
	boolean stage4start = false;
	boolean stage1 = false;
	boolean stage2 = false;
	boolean stage3 = false;
	boolean stage4 = false;
	
	ArrayList<Command> autoCommands;
	
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
			haveCAN = true;
			CAN_CoastMode();
			
		    // Competition use
			_climber = new CANTalon(5);
			_fuelIntake = new CANTalon(6);
			_fuelShooter = new CANTalon(7);
			
		} else {
			
			// Practice Robot v2 use - Talon SRX
			
			_frontLeftMotor  = new CANTalon(1);     //device IDs here (1 of 2)
			_rearLeftMotor   = new CANTalon(2);
			_frontRightMotor = new CANTalon(3);
			_rearRightMotor  = new CANTalon(4);
			
			// set all Talon SRX drive motors to Coast from software
			haveCAN = true;
			CAN_CoastMode();
			
			_climber = new Talon(0);
			_fuelIntake = new Talon(1);
			_fuelShooter = new Talon(2);
						
			// Practice Robot v1 use - Sparks
/*
			_frontLeftMotor  = new Spark(0); 		//device IDs here (1 of 2)
			_rearLeftMotor   = new Spark(1);
			_frontRightMotor = new Spark(2);
			_rearRightMotor  = new Spark(3);
			
			_climber = new Victor(5);
			_fuelIntake = new Victor(6);
			_fuelShooter = new Victor(7);
*/
			

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
		
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Default Fwd", defaultAutoFwd);
		autoChooser.addObject("Hang Gear, Fuel Goal",  autoAllianceGearHang);
		autoChooser.addObject("Auto Back", customAutoBack);
		autoChooser.addObject("Auto Shooter", customAutoShooter);
		//autoChooser.addObject("Alliance Wag", customAutoAllianceWag);
		autoChooser.addObject("Vision Chase", visionChase);
		SmartDashboard.putData("Auto modes", autoChooser);
		
		allianceChooser = new SendableChooser<String>();
		allianceChooser.addDefault("Red ", defaultAlliance);
		allianceChooser.addObject("Blue ", blueAlliance);
		SmartDashboard.putData("Alliance", allianceChooser);
		
		
		//startAutomaticCapture(cam0, 0);//string name(the one you call it by, device id number)
		
//		new Thread(() -> {
			camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
			camera.setBrightness(20);
			camera.setExposureAuto();
			camera.setExposureManual(-144);
			camera.setFPS(25);
			camera.setWhiteBalanceAuto();
			camera.setWhiteBalanceManual(3);	
            
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
//		}).start();
		
		initiateVisionThread();
        
		ahrs = new navxmxp_data_monitor();
		
		initiateTurnController();
		
		sonar = new Sonar();
		
		autoCommands = new ArrayList<Command>();
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
		autonomousMasterCounter = 0;
		
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
		
		initiateVisionThread();
		initiateTurnController ();
		rotateToAngle = false;
		if (turnController != null) {
			turnController.disable();
		}
		
		autoSelected = (String) autoChooser.getSelected();
		//String autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		allianceSelected = (String) allianceChooser.getSelected();
		System.out.println("Alliance selected: " + allianceSelected);

		switch(autoSelected) {
		case customAutoBack:
			break;
		case autoAllianceGearHang:
			//autoDriveForward(100,2.0);
			//autoTurn(1.0,-60.0);
			_drive.setSafetyEnabled(false);
			dist = new LineDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY());
			autoCommands.add(new Command("autoDriveForward",100,2.0));
			autoCommands.add(new Command("autoTurn",1.0,-6.0));
			break;
		case defaultAutoFwd:
		default:
			_drive.setSafetyEnabled(false);
			dist = new LineDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY());
			autoCommands.add(new Command("autoDriveForward",100,2.0));
			break;
		}
		
		// custom auto flags
		stage1start = false;
		stage2start = false;
		stage3start = false;
		stage4start = false;
		stage1 = false;
		stage2 = false;
		stage3 = false;
		stage4 = false;
		
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		ahrs.printStats();
		count++;
		autonomousMasterCounter++;
		
		// runClimber 2 seconds to break tape, release gear holder
		if (autonomousMasterCounter < 100) {
			//DriverStation.reportWarning("runClimber in autonomous", false);
			runClimber(0.5);
		} else {
			stopClimber();
			//DriverStation.reportWarning("stopClimber in autonomous", false);
			
		}
		if ((autonomousMasterCounter > 755) && !(autoSelected.equals(visionChase)) ){
			// 750 * 20ms = 15secs of autonomous period
			_drive.drive(0.0, 0.0);
			stopClimber();
			stopIntake();
			DriverStation.reportWarning("Autonomous routine exceeded 15 seconds", false);
			return;
		}
		
		// from right side, drive 100% forward 84 count (1.68s=121"), turn left 60deg, fwd 6 count (.125s=9")
		
		// Drive for 2 seconds
		/*
		if (timer.get() < 2.0) {
			_drive.drive(-0.5, 0.0); // drive forwards half speed
		} else {
			_drive.drive(0.0, 0.0); // stop robot
		}  */
		

		switch(autoSelected) {
		case customAutoBack:
			_drive.setSafetyEnabled(false);
			if (count < 100) {
				_drive.drive(0.5, 0.0); // spin at half speed
			} else {
				//Timer.delay(2.0);		 //    for 2 seconds
				_drive.drive(0.0, 0.0);	 // stop robot
			}
			break;
		case customAutoShooter:
			_drive.setSafetyEnabled(false);
			if (count < 700) {
				runShooter();
			} else {
				stopShooter();
			}
			break;
		case customAutoAllianceWag:
			_drive.setSafetyEnabled(false);
			break;
		case visionChase:
			_drive.setSafetyEnabled(false);
			double centerX;
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			turn = centerX - (IMG_WIDTH / 2);
//			_drive.arcadeDrive(-0.6, turn * 0.005);
			_drive.arcadeDrive(0,  turn * 0.005);
			SmartDashboard.putNumber(   "Vision Center X   ", centerX );
			SmartDashboard.putNumber(   "Vision Center turn", turn );
			break;
		case autoAllianceGearHang:
			_drive.setSafetyEnabled(false);
			//autoDriveForward(100,2.0);
			//autoTurn(1.0,-60.0);
			break;
		case defaultAutoFwd:
		default:
			_drive.setSafetyEnabled(false);
			/*
			if (!stage1start) {
				stage1start = true;
				dist = new LineDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY());
			}
			autoDriveForward(100,2.0);
			*/
			break;
		}

		autoDispatch();
	
	}

	private void autoDispatch () {
		
		if (autoCommands.isEmpty()) {
			return;
		}
		Command c = autoCommands.get(0);
		switch (c.getCommand()) {
		case "autoDriveForward":
			if (!autoDriveForward(c.getParm1(),c.getParm2()) ) {
				autoCommands.remove(0);
				dist = new LineDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY());
				count = 0;
			}
			break;
		case "autoTurn":
			if (!autoTurn(c.getParm1(),c.getParm2()) ) {
				autoCommands.remove(0);
				dist = new LineDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY());
				count = 0;
			}
		default:
			break;
				
		}
			
	}
	
	private boolean autoDriveForward (double t, double x) {
		boolean keepgoing = false;
		boolean run = false;
		if (turnController != null) {
//		if (run) {
			if (dist.getDistance(ahrs.getDisplacementX(),ahrs.getDisplacementY()) < x) {
				keepgoing = true; // keep driving straight if less than 2 meters
			}
		} else {
			if (count < t) { // spin for 2 seconds
				keepgoing = true; // keep driving straight if less than 2 seconds
			}
		}
		if (keepgoing) { 
	    	if (turnController != null) {
//			if (run) {
	    		turnController.setSetpoint(0.0f);
	    		if (!turnController.isEnabled()) {
	    			turnController.enable();
	    		}
	            //currentRotationRate = rotateToAngleRate;
	            _drive.arcadeDrive(-0.5, rotateToAngleRate);			    	
	    	} else {
				_drive.drive(-0.5, 0.0); // drive forwards half speed
	    	}
	    	
	    	return true;
		} else {
			//Timer.delay(2.0);		 //    for 2 seconds
			if (turnController != null) {
	            turnController.disable();
			}
			_drive.drive(0.0, 0.0);	 // stop robot

			return false;
		}
	}
	private boolean autoTurn (double t, double angle) {
		boolean keepgoing = false;
		if (turnController != null) {
			if (Math.abs(ahrs.getYaw()-Math.abs(angle)) > 1) {
				keepgoing = true; // keep driving straight if less than 2 meters
			}
		} else {
			if (count < t) { // spin for 2 seconds
				keepgoing = true; // keep driving straight if less than 2 seconds
			}
		}
		
		if (keepgoing) { 
	    	if (turnController != null) {
	    		turnController.setSetpoint(angle);
	    		if (!turnController.isEnabled()) {
	    			turnController.enable();
	    		}
	            //currentRotationRate = rotateToAngleRate;
	            _drive.arcadeDrive(-0.0, rotateToAngleRate);			    	
	    	} else {
				_drive.arcadeDrive(-0.0, -0.5); // drive forwards half speed with a turn
	    	}
	    	
	    	return true;
		} else {
			//Timer.delay(2.0);		 //    for 2 seconds
			if (turnController != null) {
	            turnController.disable();
			}
			_drive.drive(0.0, 0.0);	 // stop robot
			
			return false;
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
		
		initiateVisionThread();
		initiateTurnController ();
		rotateToAngle = false;
		
		if (turnController != null) {
			turnController.disable();
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		ahrs.printStats();
		sonar.getDistance();
		
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		turn = centerX - (IMG_WIDTH / 2);
//		_drive.arcadeDrive(-0.6, turn * 0.005);
//		_drive.arcadeDrive(0,  turn * 0.005);
		SmartDashboard.putNumber(   "Vision Center X   ", centerX );
		SmartDashboard.putNumber(   "Vision Center turn", turn );
		
		if(c.getPressureSwitchValue()){ //pressure switch good then set true else be false
			pressureGood = false;
		} else {
			pressureGood = true;
		}
		
		// driver input
		double forward = _gamepad.getRawAxis(1); // logitech gamepad left Y, positive is rear
    	double turn = _gamepad.getRawAxis(4); //logitech gamepad right X, positive means turn right
    	
    	SmartDashboard.putNumber(   "Driver fwd   ", forward );
    	SmartDashboard.putNumber(   "Driver turn  ", turn);
    
    	if (_gamepad.getRawButton(5)) { // left shoulder
    		CAN_BrakeMode();
    	} else {
    		CAN_CoastMode();
    	}
    	
    	if (_gamepad.getRawButton(6)) { // right shoulder
    		driveHi();
    	} else {
    		driveLo();
    	}
    	
    	if (turnController != null) {
    		
	    	if (_gamepad.getRawButton(3)) { // button X - to the left
	    		turnController.setSetpoint(-90.0f);
	    		rotateToAngle = true;
	    	}
	    	if (_gamepad.getRawButton(1)) { // button A - to the down
	    		turnController.setSetpoint(179.9f);
	    		rotateToAngle = true;
	    	}
	    	if (_gamepad.getRawButton(2)) { // button B - to the right
	    		turnController.setSetpoint(90.0f);
	    		rotateToAngle = true;
	    	}
	    	if (_gamepad.getRawButton(4)) { // button Y - to the up
	    		turnController.setSetpoint(0.0f);
	    		rotateToAngle = true;
	    	}
	        double currentRotationRate;
	        if ( rotateToAngle ) {
	            turnController.enable();
	            currentRotationRate = rotateToAngleRate;
	            _drive.arcadeDrive(forward, currentRotationRate);
	        } else {
	            turnController.disable();
	            currentRotationRate = 10.9;
	        }
	        
	        if ( Math.abs(turn) > 0.15) {
	        	turnController.disable();
	        	rotateToAngle = false;
	    	}
    	}
    	    	
    	if (_gamepad.getRawButton(1)){ // button A
    		// practice buttons
//			_rearRightMotor.set(0.1);
//			_frontRightMotor.set(0.1);
	    } else {
	    	if (! rotateToAngle) {
	    		_drive.arcadeDrive(forward, turn);
	    	}
	    }
	

    	// operator input
    	double input = _joy.getY();  // push forward is negative values
    	boolean trigger = _joy.getRawButton(1); // trigger
    	boolean shooter = _joy.getRawButton(3); // top
   	
    	double climberInput = Math.abs(input);
    	if (_joy.getRawButton(2) && (climberInput > 0.15) ){  // joystick button 2
    		
    		// rescale and limit max climber motor output to 0.7
    		climberInput = climberInput * 0.7;  
 
    		runClimber(-climberInput); // input will turn motor clockwise
    	} else {
    		stopClimber();
    		
    	}

    	if (trigger) {    // Operator trigger is Intake
    		runIntake();
    	} else {
    		stopIntake();
    	}
    	
    	if (shooter) {    
    		if (_joy.getRawButton(4)) { // shooter and button 4 together
    			runShooter(0.8,false);
    		} else {
    			runShooter();  //placeholder for test
    		}
    	} else {
    		stopShooter();
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
		
		initiateVisionThread();
		initiateTurnController ();
		rotateToAngle = false;
		
		if (turnController != null) {
			turnController.disable();
		}
	}
	
	@Override
	public void disabledPeriodic() {
		ahrs.printStats();
		sonar.getDistance();
		
		initiateVisionThread();
		initiateTurnController ();
		rotateToAngle = false;
		
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		turn = centerX - (IMG_WIDTH / 2);
//		_drive.arcadeDrive(-0.6, turn * 0.005);
//		_drive.arcadeDrive(0,  turn * 0.005);
		SmartDashboard.putNumber(   "Vision Center X   ", centerX );
		SmartDashboard.putNumber(   "Vision Center turn", turn );
		
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
	
	public void runClimber(double speed) {
		speed = Math.abs(speed);
		_climber.set(speed);       // input will turn motor clockwise		
	}
	
	public void stopClimber() {
		_climber.set(0);
	}
	
	public void runIntake() {
		_fuelIntake.set(1);
	}
	
	public void stopIntake() {
		_fuelIntake.set(0);
	}

	public void runShooter(double speed, boolean fwd) {
		speed = Math.abs(speed);
		if (!fwd) {
			speed = -1.0 * speed;
		}
		_fuelShooter.set(speed);     //placeholder for test
	}
	
	public void runShooter(double speed) {
		runShooter(speed,true);
	}

	public void runShooter() {
		runShooter(0.8);                 // doesn't matter if you put pos or neg int here
	}
	public void runShooterIntakeMode() {
		runShooter(0.8, false);          // runs it in intake mode
	}
	
	public void stopShooter() {
		runShooter(0);
	}
	
	private void CAN_CoastMode () {
		if (haveCAN) {
			((CANTalon) _frontLeftMotor).enableBrakeMode(false);
			((CANTalon) _rearLeftMotor).enableBrakeMode(false);
			((CANTalon) _frontRightMotor).enableBrakeMode(false);
			((CANTalon) _rearRightMotor).enableBrakeMode(false);
		}
	}
	private void CAN_BrakeMode () {
		if (haveCAN) {
			((CANTalon) _frontLeftMotor).enableBrakeMode(true);
			((CANTalon) _rearLeftMotor).enableBrakeMode(true);
			((CANTalon) _frontRightMotor).enableBrakeMode(true);
			((CANTalon) _rearRightMotor).enableBrakeMode(true);
		}
	}
	  @Override
	  /* This function is invoked periodically by the PID Controller, */
	  /* based upon navX-MXP yaw angle input and PID Coefficients.    */
	  public void pidWrite(double output) {
	      rotateToAngleRate = output;
	  }
	  
	  public void initiateTurnController () {
		  
		  if (ahrs.isValid() && (turnController == null)) {
			turnController = new PIDController(kP, kI, kD, kF, ahrs.getAHRS(), this);
			turnController.setInputRange(-180.0f,  180.0f);
			turnController.setOutputRange(-1.0, 1.0);
			turnController.setAbsoluteTolerance(kToleranceDegrees);
			turnController.setContinuous(true);
			
			/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		    /* tuning of the Turn Controller's P, I and D coefficients.            */
		    /* Typically, only the P value needs to be modified.                   */
		    LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		  }
	  }
	  public void initiateVisionThread() {
		  
		  if ((visionThread == null) && (camera != null)) {
			try {
			    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
			        if (!pipeline.filterContoursOutput().isEmpty()) {
			            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
			            synchronized (imgLock) {
			                centerX = r.x + (r.width / 2);
			            }
			        }
			    });
			    visionThread.start();
			} catch (RuntimeException ex ) {
				DriverStation.reportError("Error instantiating vision thread:  " + ex.getMessage(), true);
			} 
		  }
	  }

}
