/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerTiggerSubsystem;

public class IndexerTiggerCommand extends CommandBase {

    private static boolean overrideMode = false;

    private final IndexerTiggerSubsystem indexerTigger;

    private final GenericHID controller;

    /**
     * Creates a new Indexer-Tigger subsystem command.
     */
    public IndexerTiggerCommand(IndexerTiggerSubsystem indexerTigger, GenericHID controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexerTigger = indexerTigger;
        this.controller = controller;

        addRequirements(indexerTigger);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // https://contattafiles.s3.us-west-1.amazonaws.com/tnt1555/3NDo05wfBTAMS27/Indexer-Tigger%20Loop.png
        if (overrideMode) {
            // Check left trigger and bumper
            if (controller.getRawButton(Constants.INDEXER_FRONT_ROLLER_SAME_BUTTON)
                    || controller.getRawAxis(Constants.INDEXER_FRONT_ROLLER_SAME_TRIGGER) >= 0.8) {
                indexerTigger.startIndexerSameDirection();
                indexerTigger.startFrontRoller();
                // Check right trigger and bumper
            } else if (controller.getRawButton(Constants.INDEXER_FRONT_ROLLER_OPPOSITE_BUTTON)
                    || controller.getRawAxis(Constants.INDEXER_FRONT_ROLLER_OPPOSITE_TRIGGER) >= 0.8) {
                indexerTigger.startIndexerOppositeDirections();
                indexerTigger.startFrontRoller();
            } else {
                indexerTigger.stopFrontRoller();
                if (controller.getPOV() == Constants.INDEXER_ONLY_SAME_POV) {
                    indexerTigger.startIndexerSameDirection();
                } else if (controller.getPOV() == Constants.INDEXER_ONLY_OPPOSITE_POV) {
                    indexerTigger.startIndexerOppositeDirections();
                } else {
                    indexerTigger.stopIndexer();
                }
            }

            if (controller.getRawButton(Constants.OVERRIDE_RUN_TIGGER)) {
                indexerTigger.startBackRoller();
            } else {
                indexerTigger.stopBackRoller();
            }
        } else {
            if (indexerTigger.isBottomBlocked()) {
                if (indexerTigger.getPowercellCount() >= 2) {
                    indexerTigger.setPowercellCount(3);
                    indexerTigger.stopFrontRoller();
                    indexerTigger.stopBackRoller();
                } else {
                    indexerTigger.startBackRoller();
                    indexerTigger.stopFrontRoller();
                }

                if (controller.getPOV() == Constants.INDEXER_ONLY_SAME_POV) {
                    indexerTigger.startIndexerSameDirection();
                } else if (controller.getPOV() == Constants.INDEXER_ONLY_OPPOSITE_POV) {
                    indexerTigger.startIndexerOppositeDirections();
                } else {
                    indexerTigger.stopIndexer();
                }
            } else {
                if (!indexerTigger.isTopBlocked() && indexerTigger.isBackRollerRunning()) {
                    indexerTigger.stopFrontRoller();
                    if (controller.getPOV() == Constants.INDEXER_ONLY_SAME_POV) {
                        indexerTigger.startIndexerSameDirection();
                    } else if (controller.getPOV() == Constants.INDEXER_ONLY_OPPOSITE_POV) {
                        indexerTigger.startIndexerOppositeDirections();
                    } else {
                        indexerTigger.stopIndexer();
                    }

                } else {
                    if (indexerTigger.isBackRollerRunning()) {
                        indexerTigger.stopBackRoller();
                        indexerTigger.addPowercellCount();
                    }

                    // Check left trigger and bumper
                    if (controller.getRawButton(Constants.INDEXER_FRONT_ROLLER_SAME_BUTTON)
                            || controller.getRawAxis(Constants.INDEXER_FRONT_ROLLER_SAME_TRIGGER) >= 0.8) {
                        indexerTigger.startIndexerSameDirection();
                        indexerTigger.startFrontRoller();
                        // Check right trigger and bumper
                    } else if (controller.getRawButton(Constants.INDEXER_FRONT_ROLLER_OPPOSITE_BUTTON)
                            || controller.getRawAxis(Constants.INDEXER_FRONT_ROLLER_OPPOSITE_TRIGGER) >= 0.8) {
                        indexerTigger.startIndexerOppositeDirections();
                        indexerTigger.startFrontRoller();
                    } else {
                        indexerTigger.stopFrontRoller();
                        if (controller.getPOV() == Constants.INDEXER_ONLY_SAME_POV) {
                            indexerTigger.startIndexerSameDirection();
                        } else if (controller.getPOV() == Constants.INDEXER_ONLY_OPPOSITE_POV) {
                            indexerTigger.startIndexerOppositeDirections();
                        } else {
                            indexerTigger.stopIndexer();
                        }
                    }
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerTigger.stopIndexer();
        indexerTigger.stopBackRoller();
        indexerTigger.stopFrontRoller();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Get whether the controls are in override mode.
     * 
     * @return Whether the controls are in override mode
     */
    public static boolean getOverrideMode() {
        return overrideMode;
    }

    /**
     * Set whether the controls are in override mode.
     * 
     * @param override Whether the controls are in override mode
     */
    public static void setOverrideMode(boolean override) {
        overrideMode = override;
    }
}
