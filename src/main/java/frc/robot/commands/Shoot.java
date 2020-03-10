/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotlib.oi.Rumble;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerTiggerSubsystem;
import frc.robot.subsystems.Shooter;

/**
 * Shoots power cells.
 */
public class Shoot extends CommandBase {

    private static final double VELOCITY_TOLERANCE = 100;

    private final Shooter shooter;
    private final IndexerTiggerSubsystem indexerTigger;

    private final Rumble ERROR_RUMBLE;
    private final Rumble EMPTY_RUMBLE;
    private final int MAX_SHOTS;

    private boolean finish = false;

    private double targetVelocity = 0;
    private boolean velocityReached = false;
    private int shots;

    /**
     * Create a new Shoot command.
     * 
     * <p>
     * Set maxShots to {@link Integer#MAX_VALUE} to have this command shoot until
     * Tigger is empty. If maxShots is -1, this command will never terminate.
     * </p>
     * 
     * @param shooter       The shooter
     * @param indexerTigger The indexer-Tigger subsystem
     * @param maxShots      The maximum number of Power Cells to shoot
     */
    public Shoot(Shooter shooter, IndexerTiggerSubsystem indexerTigger, int maxShots) {
        this(shooter, indexerTigger, maxShots, null, null);
    }

    /**
     * Create a new Shoot command.
     * 
     * <p>
     * Set maxShots to {@link Integer#MAX_VALUE} to have this command shoot until
     * Tigger is empty. If maxShots is -1, this command will never terminate.
     * </p>
     * 
     * @param shooter       The shooter
     * @param indexerTigger The indexer-Tigger subsystem
     * @param maxShots      The maximum number of shots
     * @param errorRumble   A Rumble to run when an error occurs
     * @param emptyRumble   A Rumble to run when Tigger is empty
     */
    public Shoot(Shooter shooter, IndexerTiggerSubsystem indexerTigger, int maxShots, Rumble errorRumble,
            Rumble emptyRumble) {
        addRequirements(shooter, indexerTigger);
        this.shooter = shooter;
        this.indexerTigger = indexerTigger;
        this.MAX_SHOTS = maxShots;
        this.ERROR_RUMBLE = errorRumble;
        this.EMPTY_RUMBLE = emptyRumble;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finish = false;
        velocityReached = false;
        shots = 0;
        // Verify that there are targets
        if (shooter.hasTarget()) {
            // Verify that the shooter is not overheating
            if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
                finish = true;
                RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot!");
                if (ERROR_RUMBLE != null) {
                    ERROR_RUMBLE.execute();
                }
                return;
            }
            if (!shooter.aim()) {
                finish = true;
                if (ERROR_RUMBLE != null) {
                    ERROR_RUMBLE.execute();
                }
                return;
            }
        } else {
            finish = true;
            RobotContainer.getLogger().logError("Shoot command was executed but no target can be found");
            if (ERROR_RUMBLE != null) {
                ERROR_RUMBLE.execute();
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Check if velocity is within tolerance
        if (Math.abs(shooter.getRealVelocity() - targetVelocity) < VELOCITY_TOLERANCE) {
            // Shoot
            indexerTigger.startBackRoller();
            indexerTigger.startFrontRoller();
            indexerTigger.startIndexerSameDirection();
            velocityReached = true;
        } else {
            // Stop feeding balls immediately
            indexerTigger.stopBackRoller();
            indexerTigger.stopFrontRoller();
            indexerTigger.stopIndexer();
            // Velocity reached went from true to false - A ball was shot
            // Reduce power cell count by 1
            // But if the bottom is blocked, a new Power Cell is now at the bottom
            // So there is no net change in the number of power cells
            if (velocityReached) {
                // Reduce power cell count by 1 if there are still any
                // If it's 0 then there's a counting error
                // For now don't do anything about it
                if (indexerTigger.getPowercellCount() > 0 && !indexerTigger.isBottomBlocked()) {
                    indexerTigger.reducePowercellCount();
                }
                shots++;

                if (indexerTigger.getPowercellCount() == 0) {
                    if (EMPTY_RUMBLE != null) {
                        EMPTY_RUMBLE.execute();
                    }

                    // If the command is not set to run forever, terminate when empty
                    if (MAX_SHOTS != -1) {
                        finish = true;
                    }
                }
                // If the command is not set to run forever and enough balls were shot,
                // terminate
                if (MAX_SHOTS != -1 && shots >= MAX_SHOTS) {
                    finish = true;
                }
            }
            velocityReached = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerTigger.stopFrontRoller();
        indexerTigger.stopIndexer();
        indexerTigger.stopBackRoller();
        shooter.setVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Finish if either finish is true or Tigger is empty and the command is
        // supposed to stop
        return finish || (MAX_SHOTS != -1 && indexerTigger.getPowercellCount() == 0);
    }
}
