package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Aim;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Rotate;
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
        /**
         * No auto.
         * 
         * <ul>
         * <li>Starts: Anywhere</li>
         * <li>Ends: In the same location</li>
         * <li>Scores: Nothing</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        NONE("None"),
        /**
         * Drive forwards off the initiation line.
         * 
         * <ul>
         * <li>Starts: Anywhere</li>
         * <li>Ends: 6 feet forwards</li>
         * <li>Scores: Nothing</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        INIT_FORWARDS("Initiation Line (Forwards)"),
        /**
         * Drive in reverse off the initiation line.
         * 
         * <ul>
         * <li>Starts: Anywhere</li>
         * <li>Ends: 6 feet backwards</li>
         * <li>Scores: Nothing</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        INIT_REVERSE("Initiation Line (Reverse)"),
        /**
         * Shoot all preloaded Power Cells without aiming, then drive off the initiation
         * line.
         * 
         * <ul>
         * <li>Starts: Anywhere, lined up with the Power Port</li>
         * <li>Ends: 3 feet forward</li>
         * <li>Scores: 0-3 Power Cells</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        SHOOT_MOVE_FORWARD_NOAIM("Shoot w/o Aim & Move Forward"),
        /**
         * Aim, then shoot all preloaded Power Cells, then drive off the initiation
         * line.
         * 
         * <ul>
         * <li>Starts: Anywhere</li>
         * <li>Ends: 3 feet forward</li>
         * <li>Scores: 0-3 Power Cells</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        SHOOT_MOVE_FORWARD("Shoot & Move Forward"),
        /**
         * Drive off the initiation line, aim, and then shoot all preloaded Power Cells.
         * 
         * <ul>
         * <li>Starts: Anywhere</li>
         * <li>Ends: 3 feet forward</li>
         * <li>Scores: 0-3 Power Cells</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        MOVE_FORWARD_SHOOT("Move Forward & Shoot"),
        /**
         * Drives into the opponent's trench to steal the 2 Power Cells.
         * 
         * <ul>
         * <li>Starts: Aligned and facing opponent's trench</li>
         * <li>Ends: Right outside the trench</li>
         * <li>Scores: Nothing</li>
         * <li>Preload: Any</li>
         * </ul>
         */
        POWER_CELL_STEAL("Power Cell Steal"),
        /**
         * Attempts to score 6 Power Cells by shooting the preload and the 3 Power Cells
         * in our trench.
         * 
         * <ul>
         * <li>Starts: Aligned and facing our trench</li>
         * <li>Ends: Inside our trench, right before the Control Panel</li>
         * <li>Scores: 6 Power Cells</li>
         * <li>Preload: 3 Power Cells</li>
         * </ul>
         */
        TRENCH_6PC("Trench, 6 Power Cells"),
        /**
         * Arctos 6135's Super Secret Totally Legit 13 Power Cell Auto (SSTL13PCA).
         * 
         * <ol>
         * <li>Start auto</li>
         * <li>???</li>
         * <li>Profit</li>
         * </ol>
         * <br>
         * <ul>
         * <li>Starts: [REDACTED]</li>
         * <li>Ends: [REDACTED]</li>
         * <li>Scores: &infin; Power Cells</li>
         * <li>Preload: [REDACTED]</li>
         * </ul>
         */
        DEBUG("Arctos 6135's Super Secret Totally Legit 13 Power Cell Auto");

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
                return new DriveDistance(drivetrain, 72);
            case INIT_REVERSE:
                return new DriveDistance(drivetrain, -72);
            case SHOOT_MOVE_FORWARD:
                return new Aim(shooter).deadlineWith(new AlignToTarget(drivetrain, shooter.getLimelight()))
                        .andThen(new Shoot(shooter, indexerTigger, Integer.MAX_VALUE))
                        .andThen(new DriveDistance(drivetrain, 36));
            case MOVE_FORWARD_SHOOT:
                return new DriveDistance(drivetrain, 36)
                        .andThen(new Aim(shooter).deadlineWith(new AlignToTarget(drivetrain, shooter.getLimelight())))
                        .andThen(new Shoot(shooter, indexerTigger, Integer.MAX_VALUE));
            case SHOOT_MOVE_FORWARD_NOAIM:
                return new Shoot(shooter, indexerTigger, Integer.MAX_VALUE).andThen(new DriveDistance(drivetrain, 36));
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
            case TRENCH_6PC:
                // Run intake
                return new RunIntake(intake, 1.0, false)
                        // Drive over the first two power cells and halfway to the third
                        // Stop intake when driving stops
                        .deadlineWith(new DriveDistance(drivetrain,
                                Constants.INIT_LINE_TRENCH_DISTANCE + Constants.TRENCH_PC_DISTANCE * 2.5
                                        - Constants.ROBOT_LENGTH))
                        // Reset gyro
                        .andThen(new InstantCommand(() -> {
                            drivetrain.zeroHeading();
                        }))
                        // Align and shoot
                        .andThen(new Aim(shooter).deadlineWith(new AlignToTarget(drivetrain, shooter.getLimelight())))
                        .andThen(new Shoot(shooter, indexerTigger, 2))
                        // Re-align to trench
                        // Note rotate takes radians and goes in the opposite direction
                        .andThen(new Rotate(drivetrain, Math.toRadians(drivetrain.getHeading())))
                        // Get the last Power Cell
                        .andThen(new RunIntake(intake, 1.0, false)
                                .deadlineWith(new DriveDistance(drivetrain, Constants.TRENCH_PC_DISTANCE)))
                        // Align and shoot everything left
                        .andThen(new Aim(shooter).deadlineWith(new AlignToTarget(drivetrain, shooter.getLimelight())))
                        .andThen(new Shoot(shooter, indexerTigger, Integer.MAX_VALUE));
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
