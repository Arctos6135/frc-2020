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
    
    static double DEADZONE = 0.15;
	static boolean reverseDrive = false;
    static boolean precisionDrive = false; 
    static double rampingRate = 5; //Time in seconds to go from 0 to full throttle.

	public TeleopDrive(Drivetrain driveTrain) {
		this.driveTrain = driveTrain;
		addRequirements(driveTrain);
	}

	public static boolean isReversed() {
		return reverseDrive;
	}
	public static void setReverseDrive(boolean reverseDrive) {
		TeleopDrive.reverseDrive = reverseDrive;
	}
	public static void toggleReverseDrive() {
        reverseDrive = !reverseDrive;
    }


	public static boolean isPrecisionDrive() {
        return precisionDrive;
	}
	public static void setPrecisionDrive(boolean precisionDrive) {
        TeleopDrive.precisionDrive = precisionDrive;
	}	
	public static void togglePrecisionDrive() {
        precisionDrive = !precisionDrive;
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
			l = l / 2;
			r = r / 2;
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