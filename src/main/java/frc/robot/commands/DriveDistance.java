package frc.robot.commands;

import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * A command that drives the robot forwards or backwards a certain distance.
 */
public class DriveDistance extends FollowTrajectory {

    /**
     * Create a new DriveDistance command.
     * 
     * <p>
     * Negative distances will make the robot drive backwards.
     * </p>
     * 
     * @param drivetrain The drivetrain
     * @param distance   The distance to drive
     */
    public DriveDistance(Drivetrain drivetrain, double distance) {
        super(drivetrain, new TrapezoidalTankDriveProfile(Constants.ROBOT_SPECS, distance));
    }
}
