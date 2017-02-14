package org.usfirst.frc.team433.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
			rightHopper(); // coming from red alliance side
			break;
		case 5:
			// XXX This formerly called a method notSureYet(), I removed that
			// and replaced w/ DoNothing().
			DoNothing();
			break;
		case 6:
			leftHopper(); // coming from red alliance side
			break;
		case 7:
			crossBaseline();
			break;
		}
	}

	/**
	 * This method sets the motors to zero speed.
	 */
	@Override
	public void DoNothing() {
		this.moveRobotForward(0);
	}

	/**
	 * This method is the autonomous program to place the gear on teh right peg.
	 */
	@Override
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
	@Override
	public void centerPeg() {
		LEDRing.set(true);
		double ultraValue = ultrasonic.getVoltage();
		SmartDashboard.putNumber("Autonomous Loop", autoLoop);
		SmartDashboard.putNumber("ultrasonic", ultraValue);

		if (encoders.get() <= 120 && josh != 1) {
			auton = 1;
			moveRobotForward(.4);
			autoLoop++;
		}

		else if (encoders.get() > 120 || josh == 1) {
			josh = 1;
			try {
				double[] defaultValues = new double[5];
				defaultValues[0] = 1;

				double[] GRIPlength = contourReport.getNumberArray("height", defaultValues);
				double gripHeight = GRIPlength[0];
				double[] centerx = contourReport.getNumberArray("centerX", defaultValues);
				double centerx1 = centerx[0];
				double centerx2 = centerx[1];
				double centeravg = (centerx1 + centerx2) / 2;
				SmartDashboard.putNumber("gripheight", gripHeight);

				if (gripHeight <= 50) {
					auton = 2;
					moveRobotForward(.4);
				}
				if (centerx2 != 1) {
					if (centeravg >= 145.0) {
						moveRobotTurnRight(.125, .25);
					} else if (centeravg <= 75.0) {
						moveRobotTurnLeft(.25, .125);
					}
				}
				if (auton == 2) {
					if (getUltrasonicInches() < 2.5) {
						moveRobotForward(0);
						// Timer.delay(4);
						auton = 3;
					}
				} else if (auton == 3) {
					if (encoders.get() > 15) {
						moveRobotReverse(.4);
					} else if (encoders.get() < 15) {
						auton = 4;
					}
				}
				SmartDashboard.putNumber("ultrasonic", ultraValue);
			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array is out of Bounds" + e);
			}
		} else if (auton == 4) {
			if (navx.getAngle() < 90) {
				moveRobotTurnLeft(.4, .4);
			} else if (navx.getAngle() >= 90) {
				moveRobotForward(0);
				// encoders.reset;
				navx.reset();
				auton = 5;
			}
		} else if (auton == 5) {
			if (encoders < 5) {
				moveRobotForward(.5);
			} else if (encoders >= 5) {
				if (navx.getAngle() < 90) {
					moveRobotTurnRight(.4, .4);
				} else if (navx.getAngle() >= 90) {
					moveRobotForward(0);
					auton = 6;
					navx.reset();
					encoders.reset();
				}
			}
		} else if (auton == 6) {
			if (encoders < 5) {
				moveRobotForward(.6);
			} else {
				moveRobotForward(0);
			}
		} else {
			moveRobotForward(0);
		}
		SmartDashboard.putNumber("Auton Number", auton);
	}

	/**
	 * This method is the autonomous program to place a gear on the left peg.
	 */
	@Override
	public void leftPeg() {
		if (encoders.get() < 300) {
			this.moveRobotForward(.5);
		}

		else if (encoders.get() >= 300 || auton == 1) {
			auton = 1;
			try {
				double[] centerx = this.contourReport.getNumberArray("centerx", defaultValues);
				double centerxavg = (centerx[0] + centerx[1]) / 2;
				double[] GRIPheight = this.contourReport.getNumberArray("height", defaultValues);
				double height = GRIPheight[0];

				auton = 3;
				final double gyroValue = navx.getAngle();

				if (height < 40) {
					if (centerxavg < 60) {
						this.moveRobotTurnRight(.4, .4);
					} else if (centerxavg >= 60 && centerxavg < 120) {
						this.moveRobotForward(.5);
					}
				} else if (getUltrasonicInches() < 2.5) {
					moveRobotForward(0);
					auton = 4;
					encoders.reset();
					navx.reset();
				} else {
					this.moveRobotReverse(0);
					auton = 4;
					encoders.reset();
					navx.reset();
				}

			} catch (ArrayIndexOutOfBoundsException e) {
				System.out.println("Array123 is out of Bounds" + e);
				auton = 2;
			}
			if (auton == 2) {
				moveRobotTurnRight(.4, .4);
			}
		}

		else if (auton == 4) {
			if (encoders.get() > -5) {
				moveRobotReverse(.5);
			} else {
				moveRobotReverse(0);
				auton = 5;
			}
		} else if (auton == 5) {
			if (navx.getAngle() < 45) {
				moveRobotTurnLeft(.4, .4);
			} else if (navx.getAngle() >= 45) {
				moveRobotForward(0);
				auton = 6;
			}
		} else if (auton == 6) {
			if (encoders.get() < 10) {
				moveRobotForward(.5);
			} else if (encoders.get() >= 10) {
				moveRobotForward(0);
			}
		}
	}

	/**
	 * This method is the autonomous program to ??.
	 */
	@Override
	public void rightHopper() {
		if (encoder.get() < 5) { // distance in inches: 105
			moveRobotForward(.5);
		} else if (encoder.get() >= 5) { // reaches distance
			auton = 1;
		} else if (auton == 1) {
			if (navx.getAngle() < 90) {
				moveRobotTurnRight(.4, .4);
			} else if (navx.getangle() >= 90) {
				moveRobotForward(0);
				encoders.reset();
				auton = 2;
			}
		} else if (auton == 2) {
			if (getUltrasonicInches() > 10) {

			}
		}
	}

	/**
	 * This method is the autonomous program to ??.
	 */
	@Override
	public void leftHopper() {
		// TODO Implement this method.
	}

	/**
	 * This method is the autonomous program to move the robot across the
	 * baseline.
	 */
	@Override
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
