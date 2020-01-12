/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
	private final Drivetrain driveTrain;

	// Controller related
	static double DEADZONE = 0.15;

	// Steering or movement related
	static boolean reverseDrive = false;
	static boolean precisionDrive = false;
	static double precisionFactor = 0.5; // Percentage of default steering input when driving with precisionDrive

	// Ramping related
	static double rampingRate = 1; // Time in seconds to go from 0 to full throttle.

	public TeleopDrive(Drivetrain driveTrain) {
		this.driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	/**
	 * Get whether the drive is reversed.
	 * 
	 * @return Whether the drive is reversed
	 */
	public static boolean isReversed() {
		return reverseDrive;
	}

	/**
	 * Set whether the drive should be reversed.
	 * 
	 * @param reverseDrive Whether the drive is reversed
	 */
	public static void setReverseDrive(boolean reverseDrive) {
		TeleopDrive.reverseDrive = reverseDrive;
	}

	/**
	 * Toggle whether the drive is reversed.
	 */
	public static void toggleReverseDrive() {
		reverseDrive = !reverseDrive;
	}

	public static boolean isPrecisionDrive() {
		return precisionDrive;
	}

	/**
	 * Turn precision drive on or off.
	 * 
	 * <p>
	 * In precision drive, the motor outputs are scaled down by a configurable
	 * factor.
	 * </p>
	 * 
	 * @param precisionDrive Whether the drive is in precision mode
	 */
	public static void setPrecisionDrive(boolean precisionDrive) {
		TeleopDrive.precisionDrive = precisionDrive;
	}

	/**
	 * Toggle precision drive.
	 * 
	 * <p>
	 * In precision drive, the motor outputs are scaled down by a configurable
	 * factor.
	 * </p>
	 */
	public static void togglePrecisionDrive() {
		precisionDrive = !precisionDrive;
	}

	/**
	 * Set the drive output multiplier in precision mode.
	 * 
	 * @param factor The precision mode multiplier
	 */
	public static void setPrecisionFactor(double factor) {
		precisionFactor = factor;
	}

	/**
	 * Get the drive output multiplier in precision mode.
	 * 
	 * @return The precision mode multiplier
	 */
	public static double getPrecisionFactor() {
		return precisionFactor;
	}

	@Override
	public void initialize() {
		driveTrain.setRamping(rampingRate);
	}

	@Override
	public void execute() {
		double x = RobotContainer.driverController.getRawAxis(Constants.DRIVE_LEFT_RIGHT);
		double y = -RobotContainer.driverController.getRawAxis(Constants.DRIVE_FWD_REV);

		if (!(Math.abs(x) > DEADZONE)) {
			x = 0;
		}
		if (!(Math.abs(y) > DEADZONE)) {
			y = 0;
		}

		x = Math.copySign(x * x, x);
		y = Math.copySign(y * y, y);

		if (reverseDrive) {
			y = -y;
		}

		double l = y + x;
		double r = y - x;

		if (precisionDrive) {
			l = l * precisionFactor;
			r = r * precisionFactor;
		}

		driveTrain.setMotors(l, r);
	}

	@Override
	public void end(boolean interrupted) {
		driveTrain.setRamping(0);
		driveTrain.setMotors(0, 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}