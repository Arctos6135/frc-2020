/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.arctos6135.robotlib.oi.Rumble;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

    private final boolean stopWhenEmpty;
    private final Rumble errorRumble;
    private final Rumble emptyRumble;

    private boolean finish = false;

    private double targetVelocity = 0;
    private boolean velocityReached = false;

    /**
     * Create a new Shoot command.
     * 
     * <p>
     * If stopWhenEmpty is true, this command will terminate when Tigger is empty.
     * This will be set to false during manual operator control so that even when
     * the power cells are miscounted we can still shoot, but will be true in autos.
     * </p>
     * 
     * @param shooter       The shooter
     * @param indexerTigger The indexer-Tigger subsystem
     * @param stopWhenEmpty Whether to stop the command when Tigger is empty
     * @param errorRumble   A Rumble to run when an error occurs
     * @param emptyRumble   A Rumble to run when Tigger is empty
     */
    public Shoot(Shooter shooter, IndexerTiggerSubsystem indexerTigger, boolean stopWhenEmpty, Rumble errorRumble,
            Rumble emptyRumble) {
        addRequirements(shooter, indexerTigger);
        this.shooter = shooter;
        this.indexerTigger = indexerTigger;
        this.stopWhenEmpty = stopWhenEmpty;
        this.errorRumble = errorRumble;
        this.emptyRumble = emptyRumble;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finish = false;
        velocityReached = false;
        // Verify that there are targets
        if (shooter.getLimelight().hasValidTargets()) {
            // Verify that the shooter is not overheating
            if (!shooter.getOverheatShutoffOverride() && shooter.getMonitorGroup().getOverheatShutoff()) {
                finish = true;
                RobotContainer.getLogger().logError("Shooter is overheating, cannot shoot!");
                errorRumble.execute();
                return;
            }
            // Estimate the velocity needed to reach the target
            double distance = shooter.getLimelight().estimateDistance(Constants.LIMELIGHT_HEIGHT,
                    Constants.TARGET_HEIGHT, Constants.LIMELIGHT_ANGLE);
            try {
                targetVelocity = Shooter.getRangeTable().search(distance);
            } catch (IllegalArgumentException e) {
                finish = true;
                RobotContainer.getLogger().logError(e.getMessage());
                errorRumble.execute();
                return;
            }
            // Spin up the shooter
            shooter.setVelocity(targetVelocity);
        } else {
            finish = true;
            RobotContainer.getLogger().logError("Shoot command was executed but no target can be found");
            errorRumble.execute();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Check if velocity is within tolerance
        if (Math.abs(shooter.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE) {
            // Shoot
            indexerTigger.startBackRoller();
            velocityReached = true;
        } else {
            // Stop feeding balls immediately
            indexerTigger.stopBackRoller();
            // Velocity reached went from true to false
            // A ball was shot
            if (velocityReached) {
                // Reduce power cell count by 1 if there are still any
                // If it's 0 then there's a counting error
                // For now don't do anything about it
                if (indexerTigger.getPowercellCount() > 0) {
                    indexerTigger.reducePowercellCount();
                }
                if (indexerTigger.getPowercellCount() == 0) {
                    emptyRumble.execute();
                }
            }
            velocityReached = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerTigger.stopBackRoller();
        shooter.setVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Finish if either finish is true or Tigger is empty and the command is
        // supposed to stop
        return finish || (stopWhenEmpty && indexerTigger.getPowercellCount() == 0);
    }
}
