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
    private final Drivetrain drivetrain;

    //Drive Settings
    static boolean reverseDrive = false;
    static boolean precisionDrive = false;
    static double secondsToFull = 0; //Slowest possible ramp time is 10 seconds
    static  double DEADZONE = 0.15; //0.15 works well

    public TeleopDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
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

    @Override
    public void initialize() {
        drivetrain.setRamping(secondsToFull);    
    }

    @Override
    public void execute() {
        double x = RobotContainer.driveController.getRawAxis(Constants.DRIVE_LEFT_RIGHT);
        double y = -RobotContainer.driveController.getRawAxis(Constants.DRIVE_FWD_REV);

        if (!(Math.abs(x) > DEADZONE)) {
            x = 0;
        }
        if (!(Math.abs(y) > DEADZONE)) {
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

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
        drivetrain.setRamping(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
