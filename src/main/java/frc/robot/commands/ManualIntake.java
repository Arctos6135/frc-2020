/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;;
import frc.robot.Constants;

public class ManualIntake extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final GenericHID controller;
    private final int forwardButton;
    private final int reverseButton;

    public ManualIntake(IndexerSubsystem indexerSubsystem, GenericHID controller, int forwardButton, int reverseButton) {
        this.indexerSubsystem = indexerSubsystem;
        this.controller = controller;
        this.forwardButton = forwardButton;
        this.reverseButton = reverseButton;
        addRequirements(indexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean forward = controller.getRawButton(Constants.INTAKE_FORWARD_BUTTON);
        boolean reverse = controller.getRawButton(Constants.INTAKE_REVERSE_BUTTON);
        if(forward & !reverse){
            indexerSubsystem.setMotors(1);
        }
        else if(!forward & reverse){
            indexerSubsystem.setMotors(-1);
        }
        else{
            indexerSubsystem.setMotors(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
