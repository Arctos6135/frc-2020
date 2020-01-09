/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TalonDriveTrain;
import frc.robot.RobotContainer;

public class TeleopDrive extends CommandBase {
    private final TalonDriveTrain drivetrain;

    static double rampBandLow = 0.07;
    static double rampBandHigh = 0.03;
    static boolean rampingOn = true;

    static boolean reverseDrive = false;
    static boolean precisionDrive = false;

    public TeleopDrive(TalonDriveTrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public static boolean isReversed() {
        return reverseDrive;
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

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = RobotContainer.driveController.getRawAxis(Constants.DRIVE_LEFT_RIGHT);
        double y = -RobotContainer.driveController.getRawAxis(Constants.DRIVE_FWD_REV);

        if (!(Math.abs(x) > Constants.DEADZONE)) {
            x = 0;
        }
        if (!(Math.abs(y) > Constants.DEADZONE)) {
            y = 0;
        }

        x = Math.copySign(Math.pow(x, 4), x);
        y = Math.copySign(y * y, y);

        if (reverseDrive) {
            y = -y;
        }

        double l = y + x;
        double r = y - x;

        if (precisionDrive) {
            l /= 2;
            r /= 2;
        }

        drivetrain.setMotors(l, r);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
