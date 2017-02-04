package org.usfirst.frc.team433.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	AutonomousRobot autonomousRobot;
	RobotDrive myRobot;
	Joystick joystick;
	Joystick xboxController;
	CANTalon leftDrivetrain1 = new CANTalon(1);
	CANTalon leftDrivetrain2 = new CANTalon(2);
	CANTalon rightDrivetrain1 = new CANTalon(3);
	CANTalon rightDrivetrain2 = new CANTalon(4);
	CANTalon rightDrivetrainSlaveMotor = new CANTalon(0);
	CANTalon gearRetrievalPivot = new CANTalon(6);
	CANTalon gearRetrievalAgitator = new CANTalon(7);
	CANTalon gearRetrievalFlap = new CANTalon(8);
	CameraServer server;
	Compressor compressor = new Compressor(0);

	// Solenoid solenoidGearFloorRetriever = new Solenoid(0, 0);
	Solenoid solenoidSpeedshift = null;// new Solenoid(0, 1);
	Solenoid solenoidWhaleTailServo = null;// new Solenoid(0, 2);
	Solenoid LEDRing;// = new Solenoid(0, 7);

	int currCompressor;
	int compressoron;
	int compressoroff;
	int NORMSPEED = 100;
	int autoLoop = 0;
	NetworkTable contourReport;
	VideoCamera camFront;
	AnalogInput ultrasonic = null;// = new AnalogInput(3);

	DigitalInput autonSwitchA;// new DigitalInput(0);
	DigitalInput autonSwitchB;// new DigitalInput(1);
	DigitalInput autonSwitchC;// new DigitalInput(2);
	double[] defaultValues;

	int auton;
	int switchAFinal;
	int switchBFinal;
	int switchCFinal;
	int switBinFin;

	SendableChooser<AutonomousRobot> autoChooser;

	public Robot() {
	}

	/**
	 * This method moves the robot forward.
	 * 
	 * @param speed
	 *            A representation of the speed of the motor.
	 */
	public void moveRobotForward(double speed) {
		leftDrivetrain1.set(-speed);
		leftDrivetrain2.set(-speed);
		rightDrivetrain1.set(speed);
		rightDrivetrain2.set(speed);
	}

	/**
	 * This method moves the robot backwards.
	 * 
	 * @param speed
	 *            A representation of the speed of the motor.
	 */
	public void moveRobotReverse(double speed) {
		leftDrivetrain1.set(speed);
		leftDrivetrain2.set(speed);
		rightDrivetrain1.set(-speed);
		rightDrivetrain2.set(-speed);
	}

	/**
	 * This method turns the robot left.
	 * 
	 * @param speedLeft
	 *            A representation of the speed of the motor.
	 * @param speedRight
	 *            A representation of the speed of the motor.
	 */
	public void moveRobotTurnLeft(double speedLeft, double speedRight) {
		leftDrivetrain1.set(speedLeft);
		leftDrivetrain2.set(speedLeft);
		rightDrivetrain1.set(speedRight);
		rightDrivetrain2.set(speedRight);
	}

	/**
	 * This method turns the robot right.
	 * 
	 * @param speedLeft
	 *            A representation of the speed of the motor.
	 * @param speedRight
	 *            A representation of the speed of the motor.
	 */
	public void moveRobotTurnRight(double speedLeft, double speedRight) {
		leftDrivetrain1.set(-speedLeft);
		leftDrivetrain2.set(-speedLeft);
		rightDrivetrain1.set(-speedRight);
		rightDrivetrain2.set(-speedRight);
	}

	@Override
	public void robotInit() {
		autoChooser = new SendableChooser<AutonomousRobot>();
		autoChooser.addDefault("My Default", new AutonomousRobot(0));
		autoChooser.addObject("Right Peg", new AutonomousRobot(1));
		autoChooser.addObject("Center Peg", new AutonomousRobot(2));
		autoChooser.addObject("Left Peg", new AutonomousRobot(3));
		autoChooser.addObject("Right Hopper", new AutonomousRobot(4));
		autoChooser.addObject("Do Nothing", new AutonomousRobot(5));
		autoChooser.addObject("Left Hopper", new AutonomousRobot(6));
		autoChooser.addObject("Cross Baseline", new AutonomousRobot(7));
		SmartDashboard.putData("Autonomous Chooser", autoChooser);

		myRobot = new RobotDrive(leftDrivetrain1, leftDrivetrain2, rightDrivetrain1, rightDrivetrain2);
		rightDrivetrainSlaveMotor.changeControlMode(TalonControlMode.Follower);
		rightDrivetrainSlaveMotor.set(3);
		joystick = new Joystick(0);
		xboxController = new Joystick(1);
		auton = 0;
		contourReport = NetworkTable.getTable("GRIP/myContoursReport");
		CameraServer.getInstance().startAutomaticCapture();
	}

	/**
	 * In two locations we measured the ultra-sonic voltage and the distance in
	 * inches. First calculate slope by dividing the difference in inches by the
	 * difference in voltage measurements Use equation of a line to find the
	 * offsets. So the result is the volts converted to inches.
	 * 
	 * @return The number of inches from the ultra-sonic sensor.
	 */
	public double getUltrasonicInches() {
		// TODO What do the values 17.86 and 1.7842 represent? These should be
		// defined as static variables w/ descriptive names
		double rawVoltage = ultrasonic.getVoltage();
		return (rawVoltage * 17.86) + 1.7842;
	}

	@Override
	public void autonomousInit() {
		/*
		 * autonSwitchA = new DigitalInput(0); autonSwitchB = new
		 * DigitalInput(1); autonSwitchC = new DigitalInput(2); // FIXME
		 * switRaw1/2/3 and switBinFin are use to select which autonomous //
		 * program to run based off of the physical switches. Are we still using
		 * // these? // Remember we are using the autoChooser in robotInit() to
		 * allow the // driver to select which program to run, we then execute
		 * the selected // autonomous // program in autonomousInit(). boolean
		 * analogSwitch1 = autonSwitchA.get(); boolean analogSwitch2 =
		 * autonSwitchB.get(); boolean analogSwitch3 = autonSwitchC.get();
		 * 
		 * if (analogSwitch1) { switchAFinal = 1; } else { switchAFinal = 0; }
		 * if (analogSwitch2) { switchBFinal = 1; } else { switchBFinal = 0; }
		 * if (analogSwitch3) { switchCFinal = 1; } else { switchCFinal = 0; }
		 * 
		 * int switchFinal = (switchAFinal * 4) + (switchBFinal * 2) +
		 * switchCFinal; // XXX Added this if/else block to replace the case
		 * switch which is now // in the autonomous class. if (autonomousRobot
		 * != null) { autonomousRobot.setAutonomousProgramSelector(switchFinal);
		 * } else { autonomousRobot = new AutonomousRobot(switchFinal); }
		 * autonomousRobot.start();
		 */
		autoLoop = 0;
		auton = 0;
		autonomousRobot = autoChooser.getSelected();
		LEDRing = new Solenoid(0, 7);
		LEDRing.set(true);
		autonomousRobot.start();

	}

	@Override
	public void autonomousPeriodic() {

		processCameraImage();
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		// BASIC DRIVE CONTROL - JOYSTICK
		double stickZ = joystick.getRawAxis(2);
		double stickY = joystick.getRawAxis(1);
		double Z2norm = stickZ * (NORMSPEED / 100.0);
		double y2norm = stickY * (NORMSPEED / 100.0) + Math.signum(stickY) * 0.05;
		myRobot.arcadeDrive(y2norm, Z2norm, true);

		if (currCompressor == compressoron) {
			if (joystick.getRawButton(1)) {
				solenoidSpeedshift.set(false); // solenoid set "true" will push
												// piston in
			} else {
				solenoidSpeedshift.set(true); // solenoid set "true" will
												// retract piston
			}
		}
	}

	@Override
	public void testPeriodic() {
		LiveWindow.addActuator("Drive Talon", "Right Front", rightDrivetrain1);
		LiveWindow.addActuator("Drive Talon", "Right Rear", rightDrivetrain2);
		LiveWindow.addActuator("Drive Talon", "Left Front", leftDrivetrain1);
		LiveWindow.addActuator("Drive Talon", "Left Rear", leftDrivetrain2);
	}

	private void processCameraImage() {
		// TODO pull image from camera
		// autonomousRobot.cameraProcessor.process(null); // TODO this calls the
		// process method in
		// GripPipeline, need to
		// replace null with
		// camera output.

		// FIXME The below statement needs to insert the output of the GRIP
		// processor into the contourReport.

		// contourReport.putNumberArray("ImageMatrix",
		// autonomousRobot.cameraProcessor.filterContoursOutput());

	}

}