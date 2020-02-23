/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

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

    public static double getP() {
        return AlignToTarget.kP;
    }

    public static void setP(double kP) {
        AlignToTarget.kP = kP;
    }

    public static double getI() {
        return AlignToTarget.kI;
    }

    public static void setI(double kI) {
        AlignToTarget.kI = kI;
    }

    public static double getD() {
        return AlignToTarget.kD;
    }

    public static void setD(double kD) {
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
                    drivetrain.arcadeDrive(0, output);
                });

        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);

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

    /**
     * Get a Sendable that can be sent to Shuffleboard.
     * 
     * <p>
     * It will be sent as a PID Controller. The "F" term of the PID controller will
     * be used for the tolerance instead of the feedforward.
     * </p>
     * 
     * @return A Sendable for tuning
     */
    public static Sendable getSendable() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("PIDController");
                builder.addDoubleProperty("p", AlignToTarget::getP, AlignToTarget::setP);
                builder.addDoubleProperty("i", AlignToTarget::getI, AlignToTarget::setI);
                builder.addDoubleProperty("d", AlignToTarget::getD, AlignToTarget::setD);
                builder.addDoubleProperty("f", AlignToTarget::getTolerance, AlignToTarget::setTolerance);
            }
        };
    }
}
