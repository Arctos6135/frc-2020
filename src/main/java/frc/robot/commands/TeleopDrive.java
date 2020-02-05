/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
	private final Drivetrain drivetrain;
	private final GenericHID controller;
	private final int X_AXIS;
	private final int Y_AXIS;

	// Controller related
	private static final double DEADZONE = 0.15;

	// Steering or movement related
	private static boolean reverseDrive = false;
	private static boolean precisionDrive = false;
	private static double precisionFactor = 0.5; // Percentage of default steering input when driving with
													// precisionDrive

	// Ramping related
	private static double rampingRate = 1; // Time in seconds to go from 0 to full throttle.

	public TeleopDrive(Drivetrain drivetrain, GenericHID controller, int fwdRevAxis, int leftRightAxis) {
		this.drivetrain = drivetrain;
		this.controller = controller;
		this.Y_AXIS = fwdRevAxis;
		this.X_AXIS = leftRightAxis;
		addRequirements(drivetrain);
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

	/**
	 * Set the ramping rate.
	 * 
	 * <p>
	 * This is the number of seconds it takes to go from neutral to full throttle.
	 * Note that this doesn't immediately configure the ramping rate! The command
	 * has to be restart for it to take effect.
	 * </p>
	 * 
	 * @param rampingRate The ramping rate to set
	 */
	public static void setRampingRate(double rampingRate) {
		TeleopDrive.rampingRate = rampingRate;
	}

	/**
	 * Get the ramping rate.
	 * 
	 * <p>
	 * This is the number of seconds it takes to go from neutral to full throttle.
	 * </p>
	 * 
	 * @return The ramping rate
	 */
	public static double getRampingRate() {
		return rampingRate;
	}

	@Override
	public void initialize() {
		drivetrain.setRamping(rampingRate);
		drivetrain.getDriveBase().setDeadband(DEADZONE);
	}

	@Override
	public void execute() {
		double tmp = 0;
		if (precisionDrive) {
			tmp = drivetrain.getSpeedMultiplier();
			drivetrain.setSpeedMultiplier(precisionFactor);
		}

		// Reverse the controller input if reverseDrive is NOT true
		// Because pushing forward on the stick is a negative value
		drivetrain.getDriveBase().arcadeDrive(
				reverseDrive ? controller.getRawAxis(Y_AXIS) : -controller.getRawAxis(Y_AXIS),
				controller.getRawAxis(X_AXIS), true);

		if (precisionDrive) {
			drivetrain.setSpeedMultiplier(tmp);
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.setRamping(0);
		drivetrain.setMotors(0, 0);
		drivetrain.getDriveBase().setDeadband(0.02);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}