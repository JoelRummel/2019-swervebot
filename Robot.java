/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FRC TEAM 4855 ROBOT CODE
// 2019 GAME DESTINATION: DEEP SPACE

// Swerve-bot code: Robot

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
			
			// CONTROLLABLE CONSTANTS
	
			final double CONTROL_SPEEDREDUCTION 	= 2; 	//teleop drivetrain inputs are divided by this number when turbo is NOT engaged
			final double CONTROL_DEADZONE 	= 0.21; //minimum value before joystick inputs will be considered

			final double CONTROL_CAM_MOE = 3;	// margin of lateral error for that alignment process for the limelight
			final double CONTROL_CAM_CORRECTBOTTOM = .29;	// lowest motor value to apply strafe during limelight correction
			final double CONTROL_CAM_PROCEEDSPEED = .38;	// value at which to proceed towards target
			//=======================================
			
			// OTHER CONSTANTS
			
			final double ROBOT_WIDTH = 33;
			final double ROBOT_LENGTH = 28;
			final double ROBOT_R = Math.sqrt(Math.pow(ROBOT_LENGTH,2)+Math.pow(ROBOT_WIDTH,2));
			final double ENC_TO_DEG = 1.158333;
			final double ABS_TO_DEG = 11.244444;
			final double ENC_360 = 417;
			final double IN_TO_ENC = 10.394;
			// buttons
			final int BUTTON_A = 1;
			final int BUTTON_B = 2;
			final int BUTTON_X = 3;
			final int BUTTON_Y = 4;
			final int BUTTON_LB = 5;
			final int BUTTON_RB = 6;
			final int BUTTON_SELECT = 7;
			final int BUTTON_START = 8;
			final int BUTTON_LSTICK = 9;
			final int BUTTON_RSTICK = 10;

			// BEGINNING VARIABLES
			
			int wheelTune = 0; // Remembers what wheel we are tweaking in test mode
			boolean emergencyTank = false; // True if the robot is in emergency tank drive mode
			boolean reverseRotate = false; // ?????
			boolean driverOriented = true; // true = driver oriented, false = robot oriented
			// All for calculating wheel speed/angle, if you need to read from a motor don't pull from these
			double a, b, c, d, max, temp, rads; 
			double encoderSetpointA, encoderSetpointB, encoderSetpointC, encoderSetpointD;
			double jStr, jFwd, jRcw;
			double wheelSpeed1, wheelSpeed2, wheelSpeed3, wheelSpeed4;
			// Gradual starts/stops in teleop
			double wheelSpeedActual1 = 0, wheelSpeedActual2 = 0, wheelSpeedActual3 = 0, wheelSpeedActual4 = 0;
			Timer wheelSpeedTimer = new Timer();
			// For limelight
			boolean limelightActive = true;	// true if the robot is collecting limelight data and tracking targets
			boolean limelightSeeking = false;	// true if the limelight is currently seeking a target
			int limelightPhase = 0;	// phase for limelight correction, 0=off 1=lateral 2=proceed
			// LIMELIGHT + DATA TABLES

			NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry tx = limelightTable.getEntry("tx");	// the x offset from the crosshairs
			NetworkTableEntry ty = limelightTable.getEntry("ty");	// the y offset from the crosshairs
			NetworkTableEntry ta = limelightTable.getEntry("ta");	// the area (0-100) of the object
			NetworkTableEntry tv = limelightTable.getEntry("tv");	// 1 if object is tracking, 0 if not

			NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");	// 0 for on, 1 for off
			NetworkTableEntry camMode = limelightTable.getEntry("camMode");	// 0 for main, 1 for driver view

			double limelightX, limelightY, limelightArea;
			boolean limelightTargetFound = false;
			double limelightInputTimer = -1;

			// DEFINING HARDWARE

			// Magnetic encoders
			Encoder encoderAngle[] = {
				new Encoder(2,3),
				new Encoder(6,7),
				new Encoder(4,5),
				new Encoder(0,1)
			};

			// Define swerve wheel classes
			Wheel wheel[] = {
				new Wheel(encoderAngle[0]),
				new Wheel(encoderAngle[1]),
				new Wheel(encoderAngle[2]),
				new Wheel(encoderAngle[3])
			};
			
			// The one CIMcoder on the bot, measures distance traveled
			Encoder encoderDistance = new Encoder(8,9);
			
			// Xbox controllers
			Joystick controlDriver = new Joystick(0);
			Joystick controlOperator = new Joystick(1);
			
			// NavX Constructor
			AHRS ahrs = new AHRS(SPI.Port.kMXP); //Use this for gyro
			
			// All motors
			Spark motorAngle[] = { // Directional motors
				new Spark(7),
				new Spark(6),
				new Spark(4),
				new Spark(5)
			};
			
			Spark motorDrive[] = { // Movement motors
				new Spark(1),
				new Spark(2),
				new Spark(10),
				new Spark(0)
			};
			
			// PID LOOPS

			// These control the steering motors using the mers (?? idk what mers is)
			PIDController PIDdrive[] = {
				new PIDController(0.035,0,0.01,encoderAngle[0],motorAngle[0]),
				new PIDController(0.035,0,0.01,encoderAngle[1],motorAngle[1]),
				new PIDController(0.035,0,0.01,encoderAngle[2],motorAngle[2]),
				new PIDController(0.035,0,0.01,encoderAngle[3],motorAngle[3])
			};
			
			// Left behind in old program, unsure if they are necessary yet
			
				// These are used for autonomous in the turning to a specific angle
				PIDController PIDautoAngle[] = {
					new PIDController(0.06,0,0.013,ahrs,motorDrive[0]),
					new PIDController(0.06,0,0.013,ahrs,motorDrive[1]),
					new PIDController(0.06,0,0.013,ahrs,motorDrive[2]),
					new PIDController(0.06,0,0.013,ahrs,motorDrive[3])
				};
				
				// These are used for autonomous in moving to a certain distance
				PIDController PIDautoDistance[] = {
					new PIDController(0.025,0,0.01,encoderDistance,motorDrive[0]),
					new PIDController(0.025,0,0.01,encoderDistance,motorDrive[1]),
					new PIDController(0.025,0,0.01,encoderDistance,motorDrive[2]),
					new PIDController(0.025,0,0.01,encoderDistance,motorDrive[3])
				};
			

			// End of variable definitions
			// <--- ROBOT INITIALIZATION --->
			
			/**
			 * This function is called when the robot is turned on
			 */
			@Override
			public void robotInit() {
				
				PIDdrive[0].setOutputRange(-1, 1);
				PIDdrive[1].setOutputRange(-1, 1);
				PIDdrive[2].setOutputRange(-1, 1);
				PIDdrive[3].setOutputRange(-1, 1);
				
				//Autonomous wheel speed loop settings
				
				for (int i=0;i<=3;i++) {
					PIDautoAngle[i].setOutputRange(-0.5, 0.5);
					PIDautoAngle[i].setInputRange(-180, 180);
					PIDautoAngle[i].setContinuous();
					PIDautoDistance[i].setOutputRange(-0.2, 0.2);
				}
			}
			
			/**
			 * This function is called periodically while disabled
			 */
			public void disabledPeriodic() {
				
			}
			
			/**
			 * This function is called immediately when autonomous begins
			 */
			@Override
			public void autonomousInit() {
				wheelSpeedTimer.start();
				wheelSpeedTimer.reset();
				encoderDistance.reset();
				ahrs.reset();
			}

			/**
			 * This function is called periodically during autonomous
			 */
			@Override
			public void autonomousPeriodic() {
				
				SmartDashboard.putNumber("Gyro", ahrs.getYaw());
				SmartDashboard.putNumber("CIMCODER",encoderDistance.get());
				
				}
			
			/**
			 * This function is called when teleop begins
			 */
			public void teleopInit() {
				if (PIDautoAngle[0].isEnabled()) {
					setAllPIDControllers(PIDautoAngle, false);
				}
				if (PIDautoDistance[0].isEnabled()) {
					setAllPIDControllers(PIDautoDistance, false);
				}
				setAllPIDControllers(PIDdrive, true);
				
				encoderDistance.reset();
				
				wheelSpeedTimer.start();
				wheelSpeedTimer.reset();
				
				ahrs.reset();
				encoderAngle[0].reset();encoderAngle[1].reset();encoderAngle[2].reset();encoderAngle[3].reset();
				resetAllWheels();
			}
			
			/**
			 * This function is called immediately when the robot is disabled
			 */
			public void disabledInit() {
				setAllPIDControllers(PIDdrive, false);
				setAllPIDControllers(PIDautoAngle, false);
				setAllPIDControllers(PIDautoDistance, false);
				
				setAllPIDSetpoints(PIDdrive, 0);
			}
			
			/**
			 * This adjusts the angle of the wheels and sets their speed based on joystick/autonomous input.
			 * 
			 * @param FWD The desired forward speed of the robot
			 * @param STR The desired strafing speed of the robot
			 * @param RCW The desired rotation speed of the robot
			 */
			public void swerve(double FWD,double STR,double RCW,boolean driverOriented) {
				if (driverOriented) {
					rads = ahrs.getYaw() * Math.PI/180;
					temp = FWD*Math.cos(rads) + STR*Math.sin(rads);
					STR = -FWD*Math.sin(rads) + STR*Math.cos(rads);
					FWD = temp;
				}

				a = STR - RCW * (ROBOT_LENGTH / ROBOT_R);
				b = STR + RCW * (ROBOT_LENGTH / ROBOT_R);
				c = FWD - RCW * (ROBOT_WIDTH / ROBOT_R);
				d = FWD + RCW * (ROBOT_WIDTH / ROBOT_R);

				//1..4: front_right, front_left, rear_left, rear_right

				wheelSpeed1 = Math.sqrt(Math.pow(b,2)+Math.pow(c,2));
				wheelSpeed2 = Math.sqrt(Math.pow(b,2)+Math.pow(d,2));
				wheelSpeed3 = Math.sqrt(Math.pow(a,2)+Math.pow(d,2));
				wheelSpeed4 = Math.sqrt(Math.pow(a,2)+Math.pow(c,2));

				encoderSetpointA = wheel[0].calculateWheelAngle(b,c);
				PIDdrive[0].setSetpoint(encoderSetpointA);SmartDashboard.putNumber("Enc. A setpoint", encoderSetpointA);
				
				encoderSetpointB = wheel[1].calculateWheelAngle(b,d);
				PIDdrive[1].setSetpoint(encoderSetpointB);SmartDashboard.putNumber("Enc. B setpoint", encoderSetpointB);
				
				encoderSetpointC = wheel[2].calculateWheelAngle(a,d);
				PIDdrive[2].setSetpoint(encoderSetpointC);SmartDashboard.putNumber("Enc. C setpoint", encoderSetpointC);
				
				encoderSetpointD = wheel[3].calculateWheelAngle(a,c);
				PIDdrive[3].setSetpoint(encoderSetpointD);SmartDashboard.putNumber("Enc. D setpoint", encoderSetpointD);

				max=wheelSpeed1; if(wheelSpeed2>max)max=wheelSpeed2; if(wheelSpeed3>max)max=wheelSpeed3; if(wheelSpeed4>max)max=wheelSpeed4;
				if (max > 1) {wheelSpeed1/=max; wheelSpeed2/=max; wheelSpeed3/=max; wheelSpeed4/=max;}
				
				wheelSpeed1 *= wheel[0].getFlip();
				wheelSpeed2 *= wheel[1].getFlip();
				wheelSpeed3 *= wheel[2].getFlip();
				wheelSpeed4 *= wheel[3].getFlip();
				
				//Move[2].set(testStick.getRawAxis(1));
				
				if (wheelSpeedTimer.get()>0.1) {
					if (wheelSpeed1 - wheelSpeedActual1 > 0.1) {wheelSpeedActual1 += 0.1;} else if (wheelSpeed1 - wheelSpeedActual1 < -0.1) {wheelSpeedActual1 -= 0.1;} else {wheelSpeedActual1 = wheelSpeed1;}
					if (wheelSpeed2 - wheelSpeedActual2 > 0.1) {wheelSpeedActual2 += 0.1;} else if (wheelSpeed2 - wheelSpeedActual2 < -0.1) {wheelSpeedActual2 -= 0.1;} else {wheelSpeedActual2 = wheelSpeed2;}
					if (wheelSpeed3 - wheelSpeedActual3 > 0.1) {wheelSpeedActual3 += 0.1;} else if (wheelSpeed3 - wheelSpeedActual3 < -0.1) {wheelSpeedActual3 -= 0.1;} else {wheelSpeedActual3 = wheelSpeed3;}
					if (wheelSpeed4 - wheelSpeedActual4 > 0.1) {wheelSpeedActual4 += 0.1;} else if (wheelSpeed4 - wheelSpeedActual4 < -0.1) {wheelSpeedActual4 -= 0.1;} else {wheelSpeedActual4 = wheelSpeed4;}
					wheelSpeedTimer.reset();
				}
				//Move[0].set(wsActual1);Move[1].set(wsActual2);Move[2].set(wsActual3);Move[3].set(wsActual4);
				
				motorDrive[0].set(wheelSpeed1);motorDrive[1].set(wheelSpeed2);motorDrive[2].set(wheelSpeed3);motorDrive[3].set(wheelSpeed4);
			}
			
			/**
			 * This function is called periodically during teleop mode
			 */
			@Override
			public void teleopPeriodic() {
				
				SmartDashboard.putNumber("CIMCODER", encoderDistance.get());
				SmartDashboard.putNumber("Gyro", ahrs.getYaw());
				
				if (!emergencyTank) {
					if (!controlDriver.getRawButton(1) && !limelightSeeking) {
						driverOriented = true;
						jFwd = -controlDriver.getRawAxis(1);if (Math.abs(jFwd) < CONTROL_DEADZONE) jFwd = 0;
						if (!controlDriver.getRawButton(5)) jFwd /= CONTROL_SPEEDREDUCTION;
						jStr = controlDriver.getRawAxis(0);if (Math.abs(jStr) < CONTROL_DEADZONE) jStr = 0;
						if (!controlDriver.getRawButton(5)) jStr /= CONTROL_SPEEDREDUCTION;
						jRcw = controlDriver.getRawAxis(4);if (Math.abs(jRcw) < CONTROL_DEADZONE) jRcw = 0;
						if (!controlDriver.getRawButton(5)) jRcw /= CONTROL_SPEEDREDUCTION;
						if (reverseRotate) {jRcw=-jRcw;}
						swerve(jFwd,jStr,jRcw,driverOriented);
					} else {
						/*
						driverOriented = false;
							Auto tilt correct program from 2018, deprecated for now
						if (ahrs.getRoll() >= 2) {
							swerve(0,0.5,0,false);
						} else if (ahrs.getRoll() <= -2) {
							swerve(0,-0.5,0,false);
						} else if (ahrs.getPitch() >= 2) {
							swerve(0.5,0,0,false);
						} else if (ahrs.getPitch() <= -2) {
							swerve(-0.5,0,0,false);
						}*/
					}
				} else {
					setAllPIDSetpoints(PIDdrive, 0);
					resetAllWheels();
					motorDrive[0].set(controlDriver.getRawAxis(5));motorDrive[3].set(controlDriver.getRawAxis(5));
					motorDrive[2].set(controlDriver.getRawAxis(1));motorDrive[1].set(controlDriver.getRawAxis(1));
				}
				
				// Reset the gyroscope
				if (controlDriver.getRawButton(BUTTON_Y)) ahrs.reset();
				
				// Reset the wheels
				if (controlDriver.getRawButton(BUTTON_X)) {
					resetAllWheels();
					setAllPIDSetpoints(PIDdrive, 0);
				}
				// Toggle Limelight activity
				if (controlDriver.getRawButtonPressed(BUTTON_LB)) {
					if (limelightActive) limelightActive = false; else limelightActive = true;
				}

				SmartDashboard.putBoolean("limelightActive",limelightActive);
				SmartDashboard.putBoolean("limelightSeeking",limelightSeeking);
				// Run Limelight searching if active
				if (limelightActive == true) {
					ledMode.setNumber(0);	// turn leds on
					camMode.setNumber(0);	// set cam to low-contrast view
					limelightGather();
					if (controlDriver.getRawButtonPressed(BUTTON_RB) && limelightTargetFound == true) {
						// Set seeking on
						limelightSeeking = true;
						limelightPhase = 1;
						limelightInputTimer = 50;
						System.out.println("seeking initiated");
					}
					// Track to a target if the target is still present
					if (limelightSeeking == true && limelightTargetFound == true) {

						// Phase 1: angle flush with the robot

						if (limelightPhase == 1) {
							/*double sign = -Math.signum(limelightX);
							swerve(0,sign * Math.max(Math.abs(limelightX),CONTROL_CAM_CORRECTBOTTOM),0,false);*/
							// Omitted for now

							limelightPhase = 2;
							limelightInputTimer = 50;
						}

						// Phase 2: line up laterally with the target

						if (limelightPhase == 2) {
							
							if (limelightInputTimer > 0) {
								limelightInputTimer --;
								swerve(0,.1,0,false);
							} else swerve(0,proportionalLoop(.012,limelightX,0),0,false);

							// Proceed to next step
							if ((-CONTROL_CAM_MOE < limelightX && limelightX < CONTROL_CAM_MOE)) {
								limelightPhase = 3;
								limelightInputTimer = 50;
							}
						}

						// Phase 3: proceed towards target

						if (limelightPhase == 3) {
							
							if (limelightInputTimer > 0) {
								limelightInputTimer --; 
								swerve(.1,0,0,false);
							} else swerve(proportionalLoop(.011,limelightArea,90),0,0,false);

							// Proceed to next step
							if (limelightArea >= 70) {
								limelightPhase = 0;
								limelightSeeking = false;
								limelightInputTimer = 0;
							}
						}

					} else {
						limelightSeeking = false;
						limelightPhase = 0;
					}
				} else {
					// Limelight is not active, turn lights off
					ledMode.setNumber(1);	// turn leds off
					camMode.setNumber(1);	// set cam to driver view
				}

				SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
				SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
				SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
				SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
				
			}

			/**
			 * This function is called periodically during test mode
			 */
			@Override
			public void testPeriodic() {
				if (PIDautoAngle[0].isEnabled()) {
					setAllPIDControllers(PIDautoAngle, false);
				}
				if (PIDautoDistance[0].isEnabled()) {
					setAllPIDControllers(PIDautoDistance, false);
				}
				if (PIDdrive[0].isEnabled()) {
					setAllPIDControllers(PIDdrive, false);
				}
				
				//SmartDashboard.putNumber("AI 1", ai1.getValue());
				
				SmartDashboard.putNumber("Joystick y axis", controlDriver.getRawAxis(1));
				
				SmartDashboard.putNumber("Encoder1:", encoderAngle[0].get());
				SmartDashboard.putNumber("Encoder2:", encoderAngle[1].get());
				SmartDashboard.putNumber("Encoder3:", encoderAngle[2].get());
				SmartDashboard.putNumber("Encoder4:", encoderAngle[3].get());
				
				SmartDashboard.putNumber("NavX Pitch:", ahrs.getPitch());
				SmartDashboard.putNumber("Navx Roll:", ahrs.getRoll());
				SmartDashboard.putNumber("NavX Yaw:", ahrs.getYaw());
				SmartDashboard.putNumber("NavX Angle:", ahrs.getAngle());
				SmartDashboard.putNumber("NavX Raw X:", ahrs.getRawGyroX());
				
				if (controlDriver.getRawButton(1)) wheelTune = 0;
				if (controlDriver.getRawButton(2)) wheelTune = 1;
				if (controlDriver.getRawButton(3)) wheelTune = 2;
				if (controlDriver.getRawButton(4)) wheelTune = 3;
				
				switch (wheelTune) {
				case 0:
					if (controlDriver.getRawButton(5)) motorAngle[0].set(0.3);
					else if (controlDriver.getRawButton(6)) motorAngle[0].set(-0.3); else motorAngle[0].set(0);
					break;
				case 1:
					if (controlDriver.getRawButton(5)) motorAngle[1].set(0.3);
					else if (controlDriver.getRawButton(6)) motorAngle[1].set(-0.3); else motorAngle[1].set(0);
					break;
				case 2:
					if (controlDriver.getRawButton(5)) motorAngle[2].set(0.3);
					else if (controlDriver.getRawButton(6)) motorAngle[2].set(-0.3); else motorAngle[2].set(0);
					break;
				case 3:
					if (controlDriver.getRawButton(5)) motorAngle[3].set(0.3);
					else if (controlDriver.getRawButton(6)) motorAngle[3].set(-0.3); else motorAngle[3].set(0);
					break;
				}
		}

		/**
		 * Resets all of the SwerveWheel objects, putting them on a clean slate
		 * (eliminates flipped orientations, stacked setpoints, etc.)
		 */
		public void resetAllWheels() {
			for (int i=0;i<=3;i++) {
				wheel[i].reset();
			}
		}

		public void setAllWheels(double val) {
			for (int i=0;i<=3;i++) {
				motorDrive[i].set(val * wheel[i].getFlip());
			}
		}
		
		/**
		 * Enables or disables a given array of four PIDController objects.
		 * @param pids The array of PID Controllers to set
		 * @param enabled True to enable, false to disable
		 */
		public void setAllPIDControllers(PIDController[] pids, boolean enabled) {
			for (int i=0;i<=3;i++) {
				pids[i].setEnabled(enabled);
			}
		}
		
		/**
		 * Sets the setpoints for an array of four PIDController objects.
		 * @param pids The array of PID Controllers to set
		 * @param setpoint The setpoint
		 */
		public void setAllPIDSetpoints(PIDController[] pids, double setpoint) {
			for (int i=0;i<=3;i++) {
				pids[i].setSetpoint(setpoint);
			}
		}
		
		/**
		 * Reads and posts network table values acquired from Limelight
		 */
		public void limelightGather() {
			// Read Limelight data table values
			double limelightX = tx.getDouble(0.0);
			double limelightY = ty.getDouble(0.0);
			double limelightArea = ta.getDouble(0.0);
			double ltv = tv.getDouble(0.0);
			limelightTargetFound = false;
			if (ltv != 1.0) limelightTargetFound = false; else limelightTargetFound = true;

			// Write Limelight data table values to the dashboard
			SmartDashboard.putNumber("LimelightX", limelightX);
			SmartDashboard.putNumber("LimelightY", limelightY);
			SmartDashboard.putNumber("LimelightArea", limelightArea);
			SmartDashboard.putBoolean("LimelightTargetFound", limelightTargetFound);
		}
		/**
		 * Returns a value based on sensor inputs.
		 * 
		 * @param p - the proportional constant
		 * @param currentSensor - whatever your current sensor value is
		 * @param desiredSensor - whatever you want the sensor to become after change
		 */
		public double proportionalLoop(double p, double currentSensor, double desiredSensor) {
			return p * (currentSensor - desiredSensor);
		}
}
