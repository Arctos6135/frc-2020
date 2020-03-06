package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexerTiggerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

/**
 * A class that sets up all the autos.
 */
public class Autos {

    private SendableChooser<AutoMode> chooser;

    /**
     * Valid auto modes.
     */
    public enum AutoMode {
        NONE("None"), INIT_FORWARDS("Initiation Line (Forwards)"), INIT_REVERSE("Initiation Line (Reverse)"),
        SHOOT_MOVE_BACK("Shoot & Move Back"), MOVE_BACK_SHOOT("Move Back & Shoot"),
        SHOOT_MOVE_BACK_NOAIM("Shoot w/o Aim & Move Back"), POWER_CELL_STEAL("Power Cell Steal"), DEBUG("Debug");

        String name;

        AutoMode(String name) {
            this.name = name;
        }
    }

    /**
     * Get an auto chooser with all the available auto modes.
     * 
     * @return The auto chooser
     */
    public SendableChooser<AutoMode> getChooser() {
        return chooser;
    }

    /**
     * Get the auto command corresponding to an auto mode.
     * 
     * @param mode          The auto mode
     * @param drivetrain    The drivetrain
     * @param intake        The intake
     * @param indexerTigger The indexer-tigger subsystem
     * @param shooter       The shooter subsystem
     * @return The command for the auto mode
     */
    public Command getAuto(AutoMode mode, Drivetrain drivetrain, IntakeSubsystem intake,
            IndexerTiggerSubsystem indexerTigger, Shooter shooter) {
        switch (mode) {
            case NONE:
                return null;
            case DEBUG:
                return new DriveDistance(drivetrain, 60);
            case INIT_FORWARDS:
                return new DriveDistance(drivetrain, 36);
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -36);
            case SHOOT_MOVE_BACK:
                return new AlignToTarget(drivetrain, shooter.getLimelight())
                        .andThen(new Shoot(shooter, indexerTigger, Integer.MAX_VALUE))
                        .andThen(new DriveDistance(drivetrain, -36));
            case MOVE_BACK_SHOOT:
                return new DriveDistance(drivetrain, -36).andThen(new AlignToTarget(drivetrain, shooter.getLimelight()))
                        .andThen(new Shoot(shooter, indexerTigger, Integer.MAX_VALUE));
            case SHOOT_MOVE_BACK_NOAIM:
                return new Shoot(shooter, indexerTigger, Integer.MAX_VALUE).andThen(new DriveDistance(drivetrain, -36));
            case POWER_CELL_STEAL:
                // Run intake
                return new RunIntake(intake, 1.0, false)
                        // Drive at the same time
                        // Stop running intake after driving stops
                        .deadlineWith(new DriveDistance(drivetrain,
                                Constants.INIT_LINE_TRENCH_DISTANCE + Constants.TRENCH_DISTANCE_SHORT
                                        - Constants.ROBOT_LENGTH))
                        // Continue running intake for 2 more seconds
                        .andThen(new RunIntake(intake, 1.0, false).withTimeout(2))
                        // Drive back out of the trench
                        .andThen(new DriveDistance(drivetrain, -Constants.TRENCH_DISTANCE_SHORT));
            default:
                return new InstantCommand(() -> {
                    RobotContainer.getLogger().logError("Bad auto option " + mode);
                });
        }
    }

    /**
     * Constructor.
     */
    public Autos() {
        // TODO: Is it better to construct trajectories here, or in getAutos()?
        // getAutos() will be called in autonomousInit(), so if it takes too long, the
        // entire robot will stop. On the other hand, a lot of memory can be saved by
        // only generating the trajectories we need.

        chooser = new SendableChooser<>();
        // Add all possible modes
        for (AutoMode mode : AutoMode.class.getEnumConstants()) {
            chooser.addOption(mode.name, mode);
        }
        // Set default mode as none
        chooser.setDefaultOption(AutoMode.NONE.name, AutoMode.NONE);
    }
}
