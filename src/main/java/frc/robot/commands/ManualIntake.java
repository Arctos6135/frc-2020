/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class ManualIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final GenericHID controller;
    private final int forwardButton;
    private final int reverseButton;
    private boolean isExtended = false;

    public ManualIntake(IntakeSubsystem intakeSubsystem, GenericHID controller, int forwardButton, int reverseButton) {
        this.intakeSubsystem = intakeSubsystem;
        this.controller = controller;
        this.forwardButton = forwardButton;
        this.reverseButton = reverseButton;
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean forward = controller.getRawButton(forwardButton);
        boolean reverse = controller.getRawButton(reverseButton);
        
        //Roller Code
        if(forward & !reverse){
            intakeSubsystem.setMotors(1);
        }
        else if(!forward & reverse){
            intakeSubsystem.setMotors(-1);
        }
        else{
            intakeSubsystem.setMotors(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
