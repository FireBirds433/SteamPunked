package org.usfirst.frc.team433.robot;

import javax.xml.bind.annotation.XmlElementDecl.GLOBAL;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class Autonomous {
	CANTalon leftDrivetrain1 = new CANTalon(1);
	CANTalon leftDrivetrain2 = new CANTalon(2);
	// CANTalon leftDrivetrainSlaveMotor = new CANTalon(0);
	CANTalon rightDrivetrain1 = new CANTalon(3);
	CANTalon rightDrivetrain2 = new CANTalon(4);
	CANTalon rightDrivetrainSlaveMotor = new CANTalon(0);
	
	int autoLoop;
	Solenoid GRIPLight = new Solenoid (0, 8);
	
	NetworkTable contourReport;
	double[] defaultValues;
	private int autonSwitch = 0;
	
	public Autonomous() {
	}
	
	public Autonomous(int programSwitch) {
		this.autonSwitch = programSwitch;
	}
	
	public void start() {
		switch (autonSwitch) {
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
	
	public void DoNothing() {// all switch down
		moveRobotForward(0);
	}

	public void rightPeg() {
		GRIPLight.set(true);
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

}
