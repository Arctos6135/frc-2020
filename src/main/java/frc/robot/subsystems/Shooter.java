/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.Limelight;
import frc.robot.util.MonitoredCANSparkMaxGroup;
import frc.robot.util.RangeTable;

/**
 * The shooter subsystem.
 */
public class Shooter extends SubsystemBase {

    /**
     * A CANPIDController that implements Sendable. Appears as a PID Controller on
     * Shuffleboard.
     */
    public static class SendableCANPIDController implements Sendable {

        private CANPIDController controller;
        private double setpoint = 0;
        private boolean enabled = false;

        /**
         * Constructor.
         * 
         * @param controller The controller
         */
        public SendableCANPIDController(CANPIDController controller) {
            this.controller = controller;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PIDController");

            builder.addDoubleProperty("p", controller::getP, controller::setP);
            builder.addDoubleProperty("i", controller::getI, controller::setI);
            builder.addDoubleProperty("d", controller::getD, controller::setD);
            builder.addDoubleProperty("f", controller::getFF, controller::setFF);
            builder.addDoubleProperty("setpoint", () -> {
                return setpoint;
            }, (s) -> {
                setpoint = s;

                if (enabled) {
                    controller.setReference(setpoint, ControlType.kVelocity);
                }
            });
            builder.addBooleanProperty("enabled", () -> {
                return enabled;
            }, (e) -> {
                enabled = e;

                if (enabled) {
                    controller.setReference(setpoint, ControlType.kVelocity);
                } else {
                    controller.setReference(0, ControlType.kVelocity);
                }
            });
        }

    }

    /**
     * The lowest speed for shooting any distance.
     * 
     * <p>
     * This speed is used for pre-accelerating the shooter wheel when the shooter
     * cannot find a target.
     * </p>
     */
    public static final double BASE_SPEED = 0;

    private static final double kP = 0, kI = 0, kD = 0, kF = 0;

    private RangeTable rangeTable;

    private CANSparkMax masterMotor;
    private CANSparkMax followerMotor;
    private CANEncoder encoder;
    private CANPIDController pidController;

    private double velocity = 0;

    private Limelight limelight;

    private MonitoredCANSparkMaxGroup monitorGroup;

    private boolean protectionOverridden = false;

    /**
     * Creates a new Shooter.
     */
    public Shooter(int motor1, int motor2) {
        limelight = new Limelight();

        masterMotor = new CANSparkMax(motor1, MotorType.kBrushless);
        followerMotor = new CANSparkMax(motor2, MotorType.kBrushless);
        monitorGroup = new MonitoredCANSparkMaxGroup("Shooter", Constants.MOTOR_WARNING_TEMP,
                Constants.MOTOR_SHUTOFF_TEMP, masterMotor, followerMotor);
        encoder = masterMotor.getEncoder(EncoderType.kHallSensor, Constants.COUNTS_PER_REVOLUTION);
        pidController = masterMotor.getPIDController();

        masterMotor.setInverted(false);
        // Motors mounted in opposite to each other
        followerMotor.follow(masterMotor, true);

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kF);
        pidController.setOutputRange(-1, 1);
        pidController.setReference(0, ControlType.kVelocity);
    }

    /**
     * Load the range table for the shooter.
     * 
     * @param filename The name of the range table CSV file (in the deploy
     *                 directory)
     * @throws IOException              If an I/O error occurs
     * @throws FileNotFoundException    If the file is not found
     * @throws IllegalArgumentException If the format is incorrect
     */
    public void loadRangeTable(String filename) throws IOException {
        rangeTable = RangeTable.fromCSV(new File(Filesystem.getDeployDirectory() + File.separator + filename));
    }

    /**
     * Get the range table for the shooter.
     * 
     * <p>
     * If not loaded, this will return null.
     * </p>
     * 
     * @return The range table for the shooter
     */
    public RangeTable getRangeTable() {
        return rangeTable;
    }

    /**
     * Return the shooter's Limelight.
     * 
     * @return The Limelight
     */
    public Limelight getLimelight() {
        return limelight;
    }

    /**
     * Return whether the shooter's limelight has detected any targets.
     * 
     * @return Whether the shooter has a target
     */
    public boolean hasTarget() {
        return limelight.hasValidTargets();
    }

    /**
     * Set the velocity of the shooter to aim for the detected target.
     * 
     * <p>
     * If no targets are detected, the range table is not loaded or the target
     * distance is out of range, this method will return false and the shooter will
     * be stopped.
     * </p>
     * 
     * @return Whether aiming was successful
     */
    public boolean aim() {
        if (!hasTarget()) {
            setVelocity(0);
            return false;
        }

        // Estimate the velocity needed to reach the target
        double distance = limelight.estimateDistance(Constants.LIMELIGHT_HEIGHT, Constants.TARGET_HEIGHT,
                Constants.LIMELIGHT_ANGLE);
        try {
            setVelocity(rangeTable.search(distance));
        } catch (IllegalArgumentException | NullPointerException e) {
            // Table not loaded or out of range
            RobotContainer.getLogger()
                    .logError(e instanceof NullPointerException ? "Range table not loaded!" : e.getMessage());
            setVelocity(0);
            return false;
        }
        return true;
    }

    /**
     * Get the encoder for the shooter motors.
     * 
     * @return The encoder
     */
    public CANEncoder getEncoder() {
        return encoder;
    }

    /**
     * Get the actual velocity of the shooter in RPM.
     * 
     * @return The velocity
     * @see #getVelocitySetpoint()
     */
    public double getRealVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Get the velocity setpoint (desired velocity) of the shooter in RPM.
     * 
     * @return The velocity setpoint
     * @see #getRealVelocity()
     */
    public double getVelocitySetpoint() {
        return velocity;
    }

    /**
     * Set the velocity of the shooter in RPM.
     * 
     * @param rpm The desired velocity of the shooter
     */
    public void setVelocity(double rpm) {
        pidController.setReference(monitorGroup.getOverheatShutoff() && !protectionOverridden ? 0 : rpm,
                ControlType.kVelocity);
        velocity = rpm;
    }

    /**
     * Get the CANPIDController of the motors.
     * 
     * @return The PID Controller
     */
    public CANPIDController getPIDController() {
        return pidController;
    }

    /**
     * Get the monitored SPARK MAX group used to monitor the motors.
     * 
     * @return The monitor group
     */
    public MonitoredCANSparkMaxGroup getMonitorGroup() {
        return monitorGroup;
    }

    /**
     * Set whether the overheat shutoff is overridden.
     * 
     * <p>
     * When overridden, the motors will still be active even if the shutoff limit
     * was reached. However, callbacks will still be called and the motors will
     * still be in a "shutoff" or "warning" state.
     * </p>
     * 
     * @param override Whether the overheat shutoff should be overridden
     */
    public void setOverheatShutoffOverride(boolean override) {
        this.protectionOverridden = override;
    }

    /**
     * Return whether the overheat shutoff was overridden.
     * 
     * <p>
     * When overridden, the motors will still be active even if the shutoff limit
     * was reached. However, callbacks will still be called and the motors will
     * still be in a "shutoff" or "warning" state.
     * </p>
     * 
     * @return Whether the overheat shutoff has been overridden
     */
    public boolean getOverheatShutoffOverride() {
        return protectionOverridden;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        monitorGroup.monitorOnce();

        if (monitorGroup.getOverheatShutoff()) {
            setVelocity(0);
            masterMotor.stopMotor();
        }
    }
}
