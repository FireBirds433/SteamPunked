package org.usfirst.frc.team433.robot;

import org.opencv.core.Mat;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	AutonomousRobot autonomousRobot;
	RobotDrive myRobot;
	Joystick joystick;
	Joystick xbox;

	// compressor
	Compressor compressor;
	int currCompressor;
	int compressoron;
	int compressoroff;

	// drivetrain
	Solenoid solenoidSpeedshift = null;
	int NORMSPEED = 100;
	CANTalon rightDrivetrain1 = new CANTalon(1);
	CANTalon rightDrivetrain2 = new CANTalon(2);
	CANTalon rightDrivetrainSlaveMotor = new CANTalon(3);
	CANTalon leftDrivetrain1 = new CANTalon(4);
	CANTalon leftDrivetrain2 = new CANTalon(5);
	CANTalon leftDrivetrainSlaveMotor = new CANTalon(6);

	// hanging mechanism
	CANTalon hangMotor1 = new CANTalon(7);
	CANTalon hangMotor2 = new CANTalon(8);

	// camera
	CameraServer server;
	Solenoid LEDRing;
	double[] defaultValues;
	NetworkTable contourReport;
	VideoCamera camFront;

	// loading station retrieval
	Talon gearRetrievalPivot = new Talon(3);
	Talon gearRetrievalAgitator = new Talon(2);
	AnalogInput gearUltrasonicAgitator;
	Solenoid solenoidWhaleTailServo = null;

	DigitalInput uprightGearLimitSwitch;
	DigitalInput angledGearLimitSwitch;
	DigitalInput leftAgitatorInPosition;
	DigitalInput rightAgitatorInPosition;

	// floor gear retrieval
	Solenoid solenoidGearFloorRetrieval = null;
	Talon backGearRetrieval = new Talon(3);
	DigitalInput floorRetrievalLimitUp;
	DigitalInput floorRetrievalLimitDown;

	// autonomous
	DigitalInput autonSwitchA;
	DigitalInput autonSwitchB;
	DigitalInput autonSwitchC;
	int auton;
	int josh;
	int turnRight;
	int autoLoop;
	int switchAFinal;
	int switchBFinal;
	int switchCFinal;
	int switBinFin;

	// sensors
	AnalogInput ultrasonic = null;
	AHRS navx;

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

		joystick = new Joystick(0);
		xbox = new Joystick(1);
		compressor = new Compressor(0);

		navx = new AHRS(SerialPort.Port.kUSB);

		// drievtrain
		solenoidSpeedshift = new Solenoid(0, 0);
		myRobot = new RobotDrive(leftDrivetrain1, leftDrivetrain2, rightDrivetrain1, rightDrivetrain2);
		rightDrivetrainSlaveMotor.changeControlMode(TalonControlMode.Follower);
		rightDrivetrainSlaveMotor.set(1);
		leftDrivetrainSlaveMotor.changeControlMode(TalonControlMode.Follower);
		leftDrivetrainSlaveMotor.set(4);

		// front gear delivery
		gearUltrasonicAgitator = new AnalogInput(1);
		uprightGearLimitSwitch = new DigitalInput(1);
		angledGearLimitSwitch = new DigitalInput(6);
		leftAgitatorInPosition = new DigitalInput(4);
		rightAgitatorInPosition = new DigitalInput(5);
		solenoidWhaleTailServo = new Solenoid(0, 1);

		// floor gear retrieval
		solenoidGearFloorRetrieval = new Solenoid(0, 2);
		floorRetrievalLimitUp = new DigitalInput(2);
		floorRetrievalLimitDown = new DigitalInput(3);

		// autonomous
		auton = 0;
		josh = 0;
		autonSwitchA = new DigitalInput(7);
		autonSwitchB = new DigitalInput(8);
		autonSwitchC = new DigitalInput(9);

		// camera
		LEDRing = new Solenoid(0, 7);
		contourReport = NetworkTable.getTable("GRIP/myContoursReport");
		CameraServer.getInstance().startAutomaticCapture();

		// sensors
		ultrasonic = new AnalogInput(3);

		Thread t = new Thread(() -> {

			boolean allowCam1 = false;

			UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
			camera1.setResolution(320, 240);
			camera1.setFPS(20);
			UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
			camera2.setResolution(320, 240);
			camera2.setFPS(20);

			CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);
			CvSink cvSink2 = CameraServer.getInstance().getVideo(camera2);
			CvSource outputStream = CameraServer.getInstance().putVideo("Camera Stream", 320, 240);

			Mat image = new Mat();

			while (!Thread.interrupted()) {

				if (joystick.getRawButton(2)) {
					allowCam1 = !allowCam1;
				}

				if (allowCam1) {
					cvSink2.setEnabled(false);
					cvSink1.setEnabled(true);
					cvSink1.grabFrame(image);
				} else {
					cvSink1.setEnabled(false);
					cvSink2.setEnabled(true);
					cvSink2.grabFrame(image);
				}

				outputStream.putFrame(image);
			}

		});
		t.start();
	}

	public double getUltrasonicInches() {
		// In two locations we measured the ultrasonic voltage and the distance
		// in inches.
		// First calculate slope by dividing the difference in inches by the
		// difference in voltage measurements
		// Use equation of a line to find the offsets
		double rawVoltage = ultrasonic.getVoltage();
		return (rawVoltage * 17.86) + 1.7842; // so the result is the volts
												// converted to inches

	}

	public boolean isGearBarUp() {
		boolean result;
		double ultrasonicValue = gearUltrasonicAgitator.getVoltage();
		if (ultrasonicValue < .1) {
			result = false;
		} else {
			result = true;
		}
		return result;
	}

	public boolean isGearInPosition() {
		boolean result;
		double ultrasonicValue = gearUltrasonicAgitator.getVoltage();
		if (ultrasonicValue < .1) {
			result = true;
		} else {
			result = false;
		}
		return result;
	}

	@Override
	public void autonomousInit() {
		boolean analogSwitch1 = autonSwitchA.get();
		boolean analogSwitch2 = autonSwitchB.get();
		boolean analogSwitch3 = autonSwitchC.get();

		if (analogSwitch1) {
			switchAFinal = 1;
		} else {
			switchAFinal = 0;
		}
		if (analogSwitch2) {
			switchBFinal = 1;
		} else {
			switchBFinal = 0;
		}
		if (analogSwitch3) {
			switchCFinal = 1;
		} else {
			switchCFinal = 0;
		}

		int switchFinal = (switchAFinal * 4) + (switchBFinal * 2) + switchCFinal; // XXX
																					// Added
																					// this
																					// if/else
																					// block
																					// to
																					// replace
																					// the
																					// case
																					// switch
																					// which
																					// is
																					// now
																					// in
																					// the
																					// autonomous
																					// class.
		if (autonomousRobot != null) {
			autonomousRobot.setAutonomousProgramSelector(switchFinal);
		} else {
			autonomousRobot = new AutonomousRobot(switchFinal);
		}
		autonomousRobot.start();

		autoLoop = 0;
		auton = 0;
		autonomousRobot = autoChooser.getSelected();
		LEDRing = new Solenoid(0, 7);
		LEDRing.set(true);
		autonomousRobot.start();

	}

	@Override
	public void autonomousPeriodic() {

		boolean switRaw1 = autonSwitchA.get();// analog switch 1
		boolean switRaw2 = autonSwitchB.get();// analog switch 2
		boolean switRaw3 = autonSwitchC.get();// analog switch 3

		if (switRaw1) {
			switchAFinal = 1;
		} else {
			switchAFinal = 0;
		}

		if (switRaw2) {
			switchBFinal = 1;
		} else {
			switchBFinal = 0;
		}

		if (switRaw3) {
			switchCFinal = 1;
		} else {
			switchCFinal = 0;
		}

		int switBinFin = (switchAFinal * 4) + (switchBFinal * 2) + switchCFinal;

		switch (switBinFin) {
		case 0:
			DoNothing();
			break;
		case 1:
			rightPeg();
			break;
		case 2:
			centerPeg();
			break;
		case 3:
			rightHopper();
			break;
		case 4:
			leftPeg();
			break;
		case 5:
			centerPegandStay();
			break;
		case 6:
			leftHopper();
			break;
		case 7:
			crossBaseline();
			break;
		}
		processCameraImage();
	}

	public void DoNothing() {
		moveRobotForward(0);
	}

	public void rightPeg() {
		LEDRing.set(true);

		if (autoLoop < 120) {
			moveRobotForward(.5);
			autoLoop++;
			SmartDashboard.putNumber("autoLoop", autoLoop);
		} else if (autoLoop >= 120) {
			try {
				if (turnRight <= 1) {
					moveRobotTurnLeft(.25, .25);
				}

				double[] defaultValues = new double[5];
				defaultValues[0] = 10;
				defaultValues[1] = 10;
				defaultValues[2] = 10;
				defaultValues[3] = 10;
				defaultValues[4] = 10;
				double[] centerx = contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0] / 1;

				SmartDashboard.putNumber("centerxavg", centerxavg);
				SmartDashboard.putNumber("grip height", height);

				if (height < 25) {
					if (centerx[0] < 30) {
						moveRobotTurnRight(.15, .15);
					} else if (centerx[0] >= 30 && centerx[0] < 120) {
						moveRobotForward(.4);
					}
				} else {
					moveRobotReverse(0);
				}

				turnRight++;

			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
			}
		} else {
			moveRobotForward(0);
		}

		SmartDashboard.putNumber("Autonomous Loops", autoLoop);
		SmartDashboard.putNumber("turnRight", turnRight);
		LEDRing.set(true);

		if (autoLoop < 120) {
			moveRobotForward(.5);
			autoLoop++;
			SmartDashboard.putNumber("autoLoop", autoLoop);
		} else if (autoLoop >= 120) {
			try {
				if (turnRight <= 1) {
					moveRobotTurnLeft(.25, .25);
				}

				double[] defaultValues = new double[5];
				defaultValues[0] = 10;
				defaultValues[1] = 10;
				defaultValues[2] = 10;
				defaultValues[3] = 10;
				defaultValues[4] = 10;
				double[] centerx = contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0] / 1;

				SmartDashboard.putNumber("centerxavg", centerxavg);
				SmartDashboard.putNumber("grip height", height);

				if (height < 25) {
					if (centerx[0] < 30) {
						moveRobotTurnRight(.15, .15);
					} else if (centerx[0] >= 30 && centerx[0] < 120) {
						moveRobotForward(.4);
					}
				} else {
					moveRobotReverse(0);
				}

				turnRight++;

			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
			}
		} else {
			moveRobotForward(0);
		}

		SmartDashboard.putNumber("Autonomous Loops", autoLoop);
		SmartDashboard.putNumber("turnRight", turnRight);
	}

	public void centerPeg() {
		// Dariya and Erin project --- TBD
	}

	public void rightHopper() {
		if (autoLoop < 300) {
			moveRobotForward(.7);
		} else if (autoLoop >= 300) {
			moveRobotTurnRight(.5, .5);
		} else {
			moveRobotForward(0);
		}
	}

	public void leftPeg() {
		if (this.autoLoop < 300) {
			this.moveRobotForward(.5);
			this.autoLoop++;
		}

		if (this.autoLoop >= 300) {
			try {
				double[] centerx = this.contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = this.contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0];
				if (centerxavg < 60) {
					this.moveRobotTurnRight(.4, .4);
				} else if (centerxavg >= 60 && centerxavg < 120) {
					this.moveRobotReverse(0);
				}
				if (height < 40) {
					this.moveRobotForward(.5);
				} else {
					this.moveRobotReverse(0);
				}

			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
			}
		}
	}

	public void centerPegandStay() {
	}

	public void leftHopper() {
		if (autoLoop < 300) {
			moveRobotForward(.7);
		} else if (autoLoop >= 300) {
			moveRobotTurnLeft(.5, .5);
		} else {
			moveRobotForward(0);
		}
	}

	public void crossBaseline() {
		if (autoLoop < 300) {
			moveRobotForward(.7);
		} else {
			moveRobotForward(0);
		}
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		// drivetrain
		double stickZ = joystick.getRawAxis(2);
		double stickY = joystick.getRawAxis(1);
		double Z2norm = stickZ * (NORMSPEED / 100.0);
		double y2norm = stickY * (NORMSPEED / 100.0) + Math.signum(stickY) * 0.05;
		myRobot.arcadeDrive(y2norm, Z2norm, true);

		// speed-shifting
		if (currCompressor == compressoron) {
			if (joystick.getRawButton(1)) {
				solenoidSpeedshift.set(false); // solenoid set "true" will push
												// piston in
			} else {
				solenoidSpeedshift.set(true); // solenoid set "true" will
												// retract piston
			}
		}

		// (teleop1) whale tail piston controls
		if (xbox.getRawButton(8)) {
			solenoidWhaleTailServo.set(true);
		} else if (xbox.getRawButton(9)) {
			solenoidWhaleTailServo.set(false);
		} else {
			solenoidWhaleTailServo.set(false);
		}

		// gear box pivot
		boolean isGearboxUpright = uprightGearLimitSwitch.get();
		boolean isGearboxAngled = !angledGearLimitSwitch.get();
		double gearboxSpeed = -xbox.getRawAxis(5) / 2;

		if (isGearboxUpright) {
			if (xbox.getRawAxis(5) > 0) {
				gearRetrievalPivot.set(0);
			} else if (xbox.getRawAxis(5) < 0) {
				gearRetrievalPivot.set(gearboxSpeed);
			}
		} else if (isGearboxAngled) {
			if (xbox.getRawAxis(5) < 0) {
				gearRetrievalPivot.set(0);
			} else if (xbox.getRawAxis(5) > 0) {
				gearRetrievalPivot.set(gearboxSpeed);
			}
		} else if (!isGearboxUpright && !isGearboxAngled) {
			gearRetrievalPivot.set(gearboxSpeed);
		} else {
			gearRetrievalPivot.set(0);
		}

		// gear box agitator
		boolean agiPositionLeft = leftAgitatorInPosition.get();
		boolean agiPositionRight = rightAgitatorInPosition.get();

		if (agiPositionRight) {
			if (xbox.getRawButton(1)) {
				gearRetrievalAgitator.set(-.4);
			} else if (xbox.getRawButton(2)) {
				gearRetrievalAgitator.set(0);
			}
		} else if (agiPositionLeft) {
			if (xbox.getRawButton(1)) {
				gearRetrievalAgitator.set(0);
			} else if (xbox.getRawButton(2)) {
				gearRetrievalAgitator.set(.4);
			}
		} else {
			if (xbox.getRawButton(1)) {
				gearRetrievalAgitator.set(-.4);
			} else if (xbox.getRawButton(2)) {
				gearRetrievalAgitator.set(.4);
			}
		}

		// (teleop4) Floor Gear Retrieval
		boolean floorGearPickup = xbox.getRawAxis(1) > .05 || xbox.getRawAxis(1) < -.05;
		boolean isFloorRetrievalLimitUp = !floorRetrievalLimitUp.get();
		boolean isFloorRetrievalLimitDown = floorRetrievalLimitDown.get();

		if (isFloorRetrievalLimitUp) {
			backGearRetrieval.set(0);
		} else if (isFloorRetrievalLimitDown) {
			backGearRetrieval.set(0);
		} else if (floorGearPickup) {
			backGearRetrieval.set(xbox.getRawAxis(1));
		} else {
			backGearRetrieval.set(0);
		}

		if (xbox.getRawButton(6)) {
			solenoidGearFloorRetrieval.set(true);// arms closed
		} else if (xbox.getRawButton(7)) {
			solenoidGearFloorRetrieval.set(false);// arms open
		}

		// (teleop5) Hanger
		if (xbox.getRawButton(4)) {
			hangMotor1.set(.8);
			hangMotor2.set(.8);
		} else {
			hangMotor1.set(0);
			hangMotor2.set(0);
		}

		String alexisgreat = "hi kate :)";
		SmartDashboard.putString("Alex is Great", alexisgreat);

		SmartDashboard.putNumber("Ultrasonic Inches", getUltrasonicInches());

	}

	@Override
	public void testPeriodic() {
	}

	private void processCameraImage() {
		// TODO pull image from camera
		// autonomousRobot.cameraProcessor.process(null);
		// TODO this calls the process method in GripPipeline, need to
		// replace null with camera output.

		// FIXME The below statement needs to insert the output of the GRIP
		// processor into the contourReport.

		// contourReport.putNumberArray("ImageMatrix",
		// autonomousRobot.cameraProcessor.filterContoursOutput());

	}

}