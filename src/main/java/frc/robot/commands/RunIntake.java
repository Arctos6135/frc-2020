/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake. This command will never terminate.
 * 
 * <p>
 * Use this command with {@link Command#withTimeout(double)} as it never finishes.
 * </p>
 * <p>
 * If not already lowered, this command will first lower the intake before
 * running the rollers. It can also be set to raise the intake again when it's
 * done.
 * </p>
 */
public class RunIntake extends CommandBase {

    private final IntakeSubsystem intake;
    private final boolean raiseOnEnd;
    private final double speed;

    /**
     * Creates a new run intake command.
     * 
     * @param intake     The intake
     * @param speed      The speed to run the rollers at
     * @param raiseOnEnd If true, the intake will be raised when this command ends
     */
    public RunIntake(IntakeSubsystem intake, double speed, boolean raiseOnEnd) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        this.speed = speed;
        this.raiseOnEnd = raiseOnEnd;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setExtended(true);
        intake.setMotor(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setMotor(0);
        if (raiseOnEnd) {
            intake.setExtended(false);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
