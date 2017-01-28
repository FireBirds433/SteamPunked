package org.usfirst.frc.team433.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
// This is my test comment for GIT
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Joystick xbox;
	CANTalon leftDrivetrain1 = new CANTalon(1);
	CANTalon leftDrivetrain2 = new CANTalon(2);
	// CANTalon leftDrivetrainSlaveMotor = new CANTalon(0);
	CANTalon rightDrivetrain1 = new CANTalon(3);
	CANTalon rightDrivetrain2 = new CANTalon(4);
	CANTalon rightDrivetrainSlaveMotor = new CANTalon(0);
	CANTalon gearRetrievalPivot = new CANTalon(6);
	CANTalon gearRetrievalAgitator = new CANTalon(7);
	CANTalon gearRetrievalFlap = new CANTalon(8);
	CameraServer server;
	Compressor compressor = new Compressor(0);

	Solenoid solenoidGearFloorRetriever = new Solenoid(0, 0);
	Solenoid solenoidSpeedshift = new Solenoid(0, 1);
	Solenoid solenoidWhaleTailServo = new Solenoid(0, 2);
	Solenoid LEDRing = new Solenoid(0, 7);

	int currCompressor;
	int compressoron;
	int compressoroff;
	int NORMSPEED = 100;
	int autoLoop = 0;
	NetworkTable contourReport;
	VideoCamera camFront;
	AnalogInput ultrasonic = new AnalogInput(3);
	// AnalogInput future2 = new AnalogInput(1);
	// AnalogInput future3 = new AnalogInput(2);
	// AnalogInput future4 = new AnalogInput(4);

	DigitalInput autonSwitchA = new DigitalInput(0);
	DigitalInput autonSwitchB = new DigitalInput(1);
	DigitalInput autonSwitchC = new DigitalInput(2);
	double[] defaultValues;

	int auton;
	int switchAFinal;
	int switchBFinal;
	int switchCFinal;
	int switBinFin;

	public Robot() {
	}

	public void robotInit() {
		myRobot = new RobotDrive(leftDrivetrain1, leftDrivetrain2, rightDrivetrain1, rightDrivetrain2);
		rightDrivetrainSlaveMotor.changeControlMode(TalonControlMode.Follower);
		rightDrivetrainSlaveMotor.set(3);
		stick = new Joystick(0); // joystick
		xbox = new Joystick(1); // xbox controller
		auton = 0;
		contourReport = NetworkTable.getTable("GRIP/myContoursReport");
		CameraServer.getInstance().startAutomaticCapture();
	}

	public void moveRobotForward(double speed) {
		leftDrivetrain1.set(-speed);
		leftDrivetrain2.set(-speed);
		// leftDrivetrain3.set(-speed);
		rightDrivetrain1.set(speed);
		rightDrivetrain2.set(speed);
		// rightDrivetrain3.set(speed);
	}

	public void moveRobotReverse(double speed) {
		leftDrivetrain1.set(speed);
		leftDrivetrain2.set(speed);
		// leftDrivetrain3.set(speed);
		rightDrivetrain1.set(-speed);
		rightDrivetrain2.set(-speed);
		// rightDrivetrain3.set(-speed);
	}

	public void moveRobotTurnLeft(double speedLeft, double speedRight) {
		leftDrivetrain1.set(speedLeft);
		leftDrivetrain2.set(speedLeft);
		// leftDrivetrain3.set(speedLeft);
		rightDrivetrain1.set(speedRight);
		rightDrivetrain2.set(speedRight);
		// rightDrivetrain3.set(speedRight);
	}

	public void moveRobotTurnRight(double speedLeft, double speedRight) {
		leftDrivetrain1.set(-speedLeft);
		leftDrivetrain2.set(-speedLeft);
		// leftDrivetrain3.set(-speedLeft);
		rightDrivetrain1.set(-speedRight);
		rightDrivetrain2.set(-speedRight);
		// rightDrivetrain3.set(-speedRight);
	}

	public double getUltrasonicInches() {
		// In two locations we measured the ultrasonic voltage and the distance
		// in inches.
		// First calculate slope by dividing the difference in inches by the
		// difference in voltage measurements
		// Use equation of a line to find the offsets
		double result;
		double rawVoltage = ultrasonic.getVoltage();
		result = (rawVoltage * 17.86) + 1.7842;// so the result is the volts
												// converted to inches
		return result;
	}

	public void autonomousInit() {
		autoLoop = 0;
		auton = 0;
	}

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
			notSureYet();
			break;
		case 6:
			leftHopper();
			break;
		case 7:
			crossBaseline();
			break;
		}
	}

	public void DoNothing() {// all switch down
		moveRobotForward(0);
	}

	public void rightPeg() {
		if (autoLoop < 300) {
			moveRobotForward(.5);
			autoLoop++;
		}

		if (autoLoop >= 300) {
			try {
				double[] centerx = contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0];
				if (centerxavg < 60) {
					moveRobotTurnLeft(.4, .4);
				} else if (centerxavg >= 60 && centerxavg < 120) {
					moveRobotReverse(0);
				}
				if (height < 40) {
					moveRobotForward(.5);
				} else {
					moveRobotReverse(0);
				}
			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
			}
		}
	}

	public void centerPeg() {
		// Dariya and Erin project --- TBD
	}

	public void rightHopper() {
	}

	public void leftPeg() {
		if (autoLoop < 300) {
			moveRobotForward(.5);
			autoLoop++;
		}

		if (autoLoop >= 300) {
			try {
				double[] centerx = contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0];
				if (centerxavg < 60) {
					moveRobotTurnRight(.4, .4);
				} else if (centerxavg >= 60 && centerxavg < 120) {
					moveRobotReverse(0);
				}
				if (height < 40) {
					moveRobotForward(.5);
				} else {
					moveRobotReverse(0);
				}

			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
			}
		}
	}

	public void notSureYet() {
	}

	public void leftHopper() {
	}

	public void crossBaseline() {
	}

	public void teleopInit() {
	}

	public void teleopPeriodic() {

		// BASIC DRIVE CONTROL - JOYSTICK
		double stickZ = stick.getRawAxis(2);
		double stickY = stick.getRawAxis(1);
		double Z2norm = stickZ * (NORMSPEED / 100.0);
		double y2norm = stickY * (NORMSPEED / 100.0) + Math.signum(stickY) * 0.05;
		myRobot.arcadeDrive(y2norm, Z2norm, true);

		if (currCompressor == compressoron) {
			if (stick.getRawButton(1)) {
				solenoidSpeedshift.set(false); // solenoid set "true" will push
												// piston in
			} else {
				solenoidSpeedshift.set(true); // solenoid set "true" will
												// retract piston
			}
		}

		String alexisgreat = "hi kate :)";
	}

	public void testPeriodic() {
		LiveWindow.addActuator("Drive Talon", "Right Front", rightDrivetrain1);
		LiveWindow.addActuator("Drive Talon", "Right Rear", rightDrivetrain2);
		LiveWindow.addActuator("Drive Talon", "Left Front", leftDrivetrain1);
		LiveWindow.addActuator("Drive Talon", "Left Rear", leftDrivetrain2);
	}
}