/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    private static final double kP = 0, kI = 0, kD = 0, kF = 0;

    private CANSparkMax masterMotor;
    private CANSparkMax followerMotor;
    private CANEncoder encoder;
    private CANPIDController pidController;

    /**
     * Creates a new Shooter.
     */
    public Shooter(int motor1, int motor2) {
        masterMotor = new CANSparkMax(motor1, MotorType.kBrushless);
        followerMotor = new CANSparkMax(motor2, MotorType.kBrushless);
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
     * Get the encoder for the shooter motors.
     * 
     * @return The encoder
     */
    public CANEncoder getEncoder() {
        return encoder;
    }

    /**
     * Get the velocity of the shooter in RPM.
     * 
     * @return The velocity
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Set the velocity of the shooter in RPM.
     * 
     * @param rpm The desired velocity of the shooter
     */
    public void setVelocity(double rpm) {
        pidController.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Get the CANPIDController of the motors.
     * 
     * @return The PID Controller
     */
    public CANPIDController getPIDController() {
        return pidController;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
