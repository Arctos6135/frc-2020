/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Brake extends CommandBase {

    private final Drivetrain drivetrain;
    private IdleMode startMode;

    /**
     * Creates a new Brake.
     */
    public Brake(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startMode = drivetrain.getIdleMode();
        drivetrain.setMotorMode(IdleMode.kBrake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setMotorMode(startMode);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getLeftSpeed()) < 1.0 && Math.abs(drivetrain.getRightSpeed()) < 1.0;
    }
}
