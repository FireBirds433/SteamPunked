package org.usfirst.frc.team433.robot;

public class AutonomousRobot extends Robot {
	// Solenoid GRIPLight = new Solenoid(0, 8);
	double[] defaultValues;
	private int autonomousProgramSelector = 0;
	public GripPipeline cameraProcessor = new GripPipeline();

	public AutonomousRobot() {
	}

	public AutonomousRobot(int programSwitch) {
		this.autonomousProgramSelector = programSwitch;
	}

	public void start() {
		switch (autonomousProgramSelector) {
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
			leftPeg();
			break;
		case 4:
			rightHopper();
			break;
		case 5:
			// XXX This formerly called a method notSureYet(), I removed that
			// and replaced w/ DoNothing().
			DoNothing();
			break;
		case 6:
			leftHopper();
			break;
		case 7:
			crossBaseline();
			break;
		}
	}

	/**
	 * This method sets the motors to zero speed.
	 */
	public void DoNothing() {
		this.moveRobotForward(0);
	}

	/**
	 * This method is the autonomous program to place the gear on teh right peg.
	 */
	public void rightPeg() {

		if (this.autoLoop < 300) {
			moveRobotForward(.5);
			this.autoLoop++;
		}

		if (this.autoLoop >= 300) {
			try {
				double[] centerx = this.contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = this.contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0];
				if (centerxavg < 60) {
					this.moveRobotTurnLeft(.4, .4);
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

	/**
	 * This method is the autonomous program to place a gear on the center peg.
	 */
	public void centerPeg() {
		// TODO Dariya and Erin implement this method.
	}

	/**
	 * This method is the autonomous program to place a gear on the left peg.
	 */
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

	/**
	 * This method is the autonomous program to ??.
	 */
	public void rightHopper() {
		// TODO Implement this method.
	}

	/**
	 * This method is the autonomous program to ??.
	 */
	public void leftHopper() {
		// TODO Implement this method.
	}

	/**
	 * This method is the autonomous program to move the robot across the
	 * baseline.
	 */
	public void crossBaseline() {
		// TODO Implement this method.
	}

	/**
	 * This method is the getter for autonomousProgramSelector.
	 * 
	 * @return autonomousProgramSelector contains the integer value of which
	 *         autonomous program to execute.
	 */
	public int getAutonomousProgramSelector() {
		return autonomousProgramSelector;
	}

	/**
	 * This method is the setter for autonomousProgramSelector.
	 * 
	 * @param autonomousProgramSelector
	 *            contains the integer value of which autonomous program to
	 *            execute.
	 */
	public void setAutonomousProgramSelector(int autonomousProgramSelector) {
		this.autonomousProgramSelector = autonomousProgramSelector;
	}

}
