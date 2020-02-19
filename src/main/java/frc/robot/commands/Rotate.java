package frc.robot.commands;

import com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveRotationProfile;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/**
 * A command that rotates the robot in place a set angle.
 */
public class Rotate extends FollowTrajectory {

    /**
     * Create a new Rotate command.
     * 
     * <p>
     * The angle is in <strong><em>radians</em></strong>, with <strong><em>positive
     * angles meaning counterclockwise turns (left turns)</em></strong>.
     * </p>
     * 
     * @param drivetrain The drivetrain
     * @param angle      The angle to rotate (see above for details)
     */
    public Rotate(Drivetrain drivetrain, double angle) {
        super(drivetrain, new TrapezoidalTankDriveRotationProfile(Constants.ROBOT_SPECS, angle));
    }
}
