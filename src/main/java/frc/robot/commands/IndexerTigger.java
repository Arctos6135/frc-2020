/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerTiggerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerTigger extends CommandBase {

    private final IndexerTiggerSubsystem indexerTigger;
    private final IntakeSubsystem intake;

    /**
    * Creates a new IndexerTigger.
    */
    public IndexerTigger(IndexerTiggerSubsystem indexerTigger, IntakeSubsystem intake) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexerTigger = indexerTigger;
        this.intake = intake;
        
        addRequirements(indexerTigger);
    }

    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // The Indexer and Tigger's front roller will run when the pistons of Intake is extended.
        // If the bottom sensor is activated before Tigger reaches its capaticy, a new powercell was added.
        // The back roller will bring powercells up when there is space in Tigger.
        if (intake.getPistons()) {
            indexerTigger.startIndexer();
            indexerTigger.startFrontRoller();

            if (indexerTigger.getPowercellCount() <= 2 && indexerTigger.isBottomBlocked()) {
                if ((indexerTigger.getPowercellCount() == 1) || (indexerTigger.getPowercellCount() == 0 && indexerTigger.isTopBlocked() == false))  {
                    indexerTigger.startBackRoller();
                }
                indexerTigger.addPowercellCount();
            }
        }
        else {
            indexerTigger.stopIndexer();
            indexerTigger.stopFrontRoller();
            indexerTigger.startBackRoller();
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
}
