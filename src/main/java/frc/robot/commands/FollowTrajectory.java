/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicTankDriveFollower;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.Follower.AdvancedPositionSource;
import com.arctos6135.robotpathfinder.follower.FollowerRunner;
import com.arctos6135.robotpathfinder.follower.SimpleFollowerRunner;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveGains;
import com.arctos6135.robotpathfinder.follower.TankDriveFollower.TankDriveRobot;
import com.arctos6135.stdplug.api.datatypes.PIDVADPGains;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * A {@link Command} that follows any RobotPathfinder {@link Followable
 * Followable&lt;TankDriveMoment&gt;}.
 * 
 * If the profile provided for following is an instance of
 * {@link DynamicFollowable}, then this Command will automatically use a
 * {@link DynamicTankDriveFollower} instead of a regular
 * {@link TankDriveFollower}.
 */
public class FollowTrajectory extends CommandBase {

    private static final PIDVADPGains gains = new PIDVADPGains(0, 0, 0, 0, 0, 0);
    private static double updateDelay = 0.25;

    private final Drivetrain drivetrain;

    private final TankDriveRobot robot;

    private final Followable<TankDriveMoment> profile;
    private volatile Follower<TankDriveMoment> follower;
    private final FollowerRunner runner;

    /**
     * Get the set of gains used to follow trajectories.
     * 
     * @return The gains
     */
    public static PIDVADPGains getGains() {
        return gains;
    }

    /**
     * Get the update delay for dynamic trajectories.
     * 
     * @return The update delay
     */
    public static double getUpdateDelay() {
        return updateDelay;
    }

    /**
     * Set the update delay for dynamic trajectories.
     * 
     * @param updateDelay The update delay
     */
    public static void setUpdateDelay(double updateDelay) {
        FollowTrajectory.updateDelay = updateDelay;
    }

    /**
     * Converts the Shuffleboard StdPlug gains to RobotPathfinder gains.
     * 
     * @return The RobotPathfinder TankDriveGains
     */
    private static TankDriveGains getRobotPathfinderGains() {
        return new TankDriveGains(gains.getV(), gains.getA(), gains.getP(), gains.getI(), gains.getD(), gains.getDP());
    }

    /**
     * Creates a new FollowTrajectory.
     */
    public FollowTrajectory(Drivetrain drivetrain, Followable<TankDriveMoment> profile) {
        this.drivetrain = drivetrain;
        this.profile = profile;
        addRequirements(drivetrain);

        runner = new SimpleFollowerRunner();
        robot = new TankDriveRobot(drivetrain::setLeftMotor, drivetrain::setRightMotor,
                new FunctionAdvancedPositionSource(drivetrain::getLeftSpeed, drivetrain::getLeftDistance),
                new FunctionAdvancedPositionSource(drivetrain::getRightSpeed, drivetrain::getRightDistance),
                Timer::getFPGATimestamp, drivetrain::getHeading);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(profile instanceof DynamicFollowable) {
            DynamicFollowable<TankDriveMoment> dynamicProfile = (DynamicFollowable<TankDriveMoment>) profile;

            follower = new DynamicTankDriveFollower(dynamicProfile, robot, getRobotPathfinderGains(), updateDelay);
        }
        else {
            follower = new TankDriveFollower(profile, robot, getRobotPathfinderGains());
        }

        drivetrain.setMotorMode(IdleMode.kBrake);
        runner.start(follower, 100);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
        drivetrain.setMotorMode(IdleMode.kCoast);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return runner.isFinished();
    }

    /**
     * An implementation of {@link AdvancedPositionSource} using two functions.
     */
    private static class FunctionAdvancedPositionSource implements AdvancedPositionSource {

        private Supplier<Double> positionFunc;
        private Supplier<Double> velocityFunc;

        /**
         * Create a new instance from a position and velocity supplier function.
         * 
         * @param positionFunc The position supplier
         * @param velocityFunc The velocity supplier
         */
        public FunctionAdvancedPositionSource(Supplier<Double> positionFunc, Supplier<Double> velocityFunc) {
            this.positionFunc = positionFunc;
            this.velocityFunc = velocityFunc;
        }

        @Override
        public double getPosition() {
            return positionFunc.get();
        }

        @Override
        public double getVelocity() {
            return velocityFunc.get();
        }

        @Override
        public double getAcceleration() {
            // Maintenance Note: Because in the current version of RobotPathfinder, none of
            // the dynamic motion profiles actually require a correct acceleration to update
            // (since they're all trapezoidal anyways), we can simply return 0 here for
            // convenience. In the future, this may need to be changed.
            return 0;
        }
    }
}
