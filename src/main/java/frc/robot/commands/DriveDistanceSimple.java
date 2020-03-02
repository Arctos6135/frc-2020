package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * A command that drives the robot forwards or backwards a certain distance
 * without the use of PIDs.
 */
public class DriveDistanceSimple extends CommandBase {

    private final double distance;
    private final Drivetrain drivetrain;
    private final double speed;
    private double initDistanceL;
    private double initDistanceR;

    /**
     * Create a new DriveDistanceSimple command.
     * 
     * <p>
     * Negative distances will make the robot drive backwards.
     * </p>
     * 
     * @param drivetrain The drivetrain
     * @param distance   The distance to drive
     * @param speed      The speed to drive at
     */
    public DriveDistanceSimple(Drivetrain drivetrain, double distance, double speed) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.speed = speed;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        initDistanceL = drivetrain.getLeftDistance();
        initDistanceR = drivetrain.getRightDistance();
    }

    @Override
    public void execute() {
        if (Math.abs(drivetrain.getLeftDistance() - initDistanceL) > Math.abs(distance)) {
            drivetrain.setLeftMotor(0);
        } else {
            drivetrain.setLeftMotor(distance > 0 ? speed : -speed);
        }

        if (Math.abs(drivetrain.getRightDistance() - initDistanceR) > Math.abs(distance)) {
            drivetrain.setRightMotor(0);
        } else {
            drivetrain.setRightMotor(distance > 0 ? speed : -speed);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftDistance() - initDistanceL) > Math.abs(distance)
                && Math.abs(drivetrain.getRightDistance() - initDistanceR) > Math.abs(distance);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotors(0, 0);
    }
}
