/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

/**
 * Align the robot to the target using a PID loop and a Limelight.
 */
public class AlignToTarget extends PIDCommand {

    private Drivetrain drivetrain;
    private Limelight limelight;

    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double tolerance = 0;

    public static double getKP() {
        return AlignToTarget.kP;
    }

    public static void setKP(double kP) {
        AlignToTarget.kP = kP;
    }

    public static double getKI() {
        return AlignToTarget.kI;
    }

    public static void setKI(double kI) {
        AlignToTarget.kI = kI;
    }

    public static double getKD() {
        return AlignToTarget.kD;
    }

    public static void setKD(double kD) {
        AlignToTarget.kD = kD;
    }

    public static double getTolerance() {
        return tolerance;
    }

    public static void setTolerance(double tolerance) {
        AlignToTarget.tolerance = tolerance;
    }

    /**
     * Creates a new align to target command.
     * 
     * @param drivetrain The robot drivetrain
     * @param limelight  The limelight
     */
    public AlignToTarget(Drivetrain drivetrain, Limelight limelight) {
        super(
                // The controller that the command will use
                new PIDController(kP, kI, kD),
                // This should return the measurement
                () -> limelight.getHorizontalAngle(),
                // This should return the setpoint (can also be a constant)
                () -> 0,
                // This uses the output
                output -> {
                    drivetrain.setMotors(output, -output);
                });

        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(this.drivetrain, this.limelight);

        PIDController controller = getController();
        controller.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        // Reset the PID values
        getController().setP(kP);
        getController().setI(kI);
        getController().setD(kD);
        super.initialize();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !limelight.hasValidTargets() || getController().atSetpoint();
    }
}
