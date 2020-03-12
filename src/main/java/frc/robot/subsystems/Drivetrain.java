/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MonitoredCANSparkMaxGroup;

public class Drivetrain extends SubsystemBase {

    private double leftLastRate = 0, rightLastRate = 0;
    private double lastTime;

    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightFollowerMotor;
    private final CANSparkMax leftFollowerMotor;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;

    private final AHRS ahrs;
    
    private final MonitoredCANSparkMaxGroup monitorGroup;

    private boolean protectionOverridden = false;

    // Having a speed multiplier allows for easy adjustment of top speeds
    private double speedMultiplier = 1.0;

    /**
     * Set the speed multiplier
     * 
     * <p>
     * All set speed operations are multiplied by this first before being passed to
     * the motors.
     * </p>
     * 
     * @param multiplier The speed multiplier
     */
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }

    /**
     * Get the speed multiplier.
     * 
     * <p>
     * All set speed operations are multiplied by this first before being passed to
     * the motors.
     * </p>
     * 
     * @return The speed multiplier
     */
    public double getSpeedMultiplier() {
        return speedMultiplier;
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
     * Sets the left and right side motors of the drivetrain. The input values are
     * first multiplied by the speed multiplier (see
     * {@link #setSpeedMultiplier(double)}), and then constrained to [-1, 1].
     * 
     * @param left  The left side motors percent output (inverted)
     * @param right The right side motors percent output
     */
    public void setMotors(double left, double right) {
        // Inverted in constructor
        setLeftMotor(left);
        setRightMotor(right);
    }

    /**
     * Set the percentage output of the left motor.
     * 
     * @param output The motor output
     * @see #setMotors(double, double)
     */
    public void setLeftMotor(double output) {
        leftMotor.set(monitorGroup.getOverheatShutoff() && !protectionOverridden ? 0 : output * speedMultiplier);
    }

    /**
     * Set the percentage output of the right motor.
     * 
     * @param output The motor output
     * @see #setMotors(double, double)
     */
    public void setRightMotor(double output) {
        rightMotor.set(monitorGroup.getOverheatShutoff() && !protectionOverridden ? 0 : output * speedMultiplier);
    }

    /**
     * Set the ramping rate of the motors.
     * 
     * @param rate The number of seconds from neutral to full speed
     */
    public void setRamping(double rate) {
        leftMotor.setOpenLoopRampRate(rate);
        rightMotor.setOpenLoopRampRate(rate);
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

    // Encoders
    /**
     * Reset the left and right encoders
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    /**
     * Gets the distance travelled by the left encoder, using the
     * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of
     * the {@code Encoder} class
     * 
     * @return The distance travelled by the left encoder
     */
    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    /**
     * Gets the distance travelled by the right encoder, using the
     * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of
     * the {@code Encoder} class
     * 
     * @return The distance travelled by the right encoder
     */
    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    /**
     * Gets the speed reading of the left encoder, using the
     * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the
     * {@code Encoder} class
     * 
     * @return The speed of the left encoder
     */
    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    /**
     * Gets the speed reading of the right encoder, using the
     * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the
     * {@code Encoder} class
     * 
     * @return The speed of the right encoder
     */
    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    /**
     * Calculates the acceleration of both sides.<br>
     * <br>
     * Instead of being read directly from the encoder, the acceleration is
     * calculated by deriving the result of {@link #getLeftSpeed()} and
     * {@link #getRightSpeed()}. The derivation is done only when this method is
     * called, therefore the result will be more accurate if this method was called
     * a short time ago. However, please note that two subsequent calls right after
     * each other may yield a result of 0, as the last known encoder rate from
     * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} may not have time
     * to update.
     * 
     * @return An array containing the acceleration of both sides, with element 0
     *         being the left and element 1 being the right
     */
    public double[] getAccelerations() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        double leftRate = leftEncoder.getVelocity();
        double rightRate = rightEncoder.getVelocity();
        double leftAccel = (leftRate - leftLastRate) / dt;
        double rightAccel = (rightRate - rightLastRate) / dt;
        leftLastRate = leftRate;
        rightLastRate = rightRate;
        lastTime = Timer.getFPGATimestamp();
        return new double[] { leftAccel, rightAccel };
    }

    private IdleMode idleMode = IdleMode.kCoast; // sreela

    /**
     * Sets the idle mode (brake or coast) of all the drivetrain motors.
     */
    public void setMotorMode(IdleMode mode) {
        idleMode = mode;
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);
    }

    /**
     * Gets the neutral mode of all the drivetrain motors.
     */
    public IdleMode getIdleMode() {
        return idleMode;
    }

    /**
     * Return the heading of the robot.
     * 
     * @return The heading of the robot in degrees
     */
    public double getHeading() {
        return ahrs.getFusedHeading();
    }

    /**
     * Reset the heading of the robot.
     */
    public void zeroHeading() {
        ahrs.reset();
    }

    /**
     * Get the robot's Attitude and Heading Reference System (NavX IMU).
     * 
     * @return The AHRS
     */
    public AHRS getAHRS() {
        return ahrs;
    }

    /**
     * Drives the robot with forwards/backwards translation and rotation.
     * 
     * @param translation The translation ([-1.0, 1.0])
     * @param rotation    The rotation ([-1.0, 1.0])
     */
    public void arcadeDrive(double translation, double rotation) {
        arcadeDrive(translation, rotation, 1.0);
    }

    /**
     * Drives the robot with forwards/backwards translation and rotation.
     * 
     * <p>
     * The values will be scaled by the scaling factor before given to the motors. A
     * negative rotation value rotates the robot counterclockwise.
     * </p>
     * 
     * @param translation   The translation ([-1.0, 1.0])
     * @param rotation      The rotation ([-1.0, 1.0])
     * @param scalingFactor The scaling factor to multiply the output by
     */
    public void arcadeDrive(double translation, double rotation, double scalingFactor) {
        double l = (translation + rotation) * scalingFactor;
        double r = (translation - rotation) * scalingFactor;

        setMotors(l, r);
    }

    /**
     * Writes the settings of all Spark Max motor controllers to flash.
     */
    public void burnFlash() {
        leftMotor.burnFlash();
        rightMotor.burnFlash();
        leftFollowerMotor.burnFlash();
        rightFollowerMotor.burnFlash();
    }

    /**
     * Creates a new drivetrain.
     */
    public Drivetrain(int leftMaster, int leftFollower, int rightMaster, int rightFollower) {
        ahrs = new AHRS(I2C.Port.kOnboard);
        rightMotor = new CANSparkMax(rightMaster, MotorType.kBrushless);
        leftMotor = new CANSparkMax(leftMaster, MotorType.kBrushless);
        rightFollowerMotor = new CANSparkMax(rightFollower, MotorType.kBrushless);
        leftFollowerMotor = new CANSparkMax(leftFollower, MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, Constants.COUNTS_PER_REVOLUTION);
        leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, Constants.COUNTS_PER_REVOLUTION);
        monitorGroup = new MonitoredCANSparkMaxGroup("Drivetrain", Constants.MOTOR_WARNING_TEMP,
                Constants.MOTOR_SHUTOFF_TEMP, leftMotor, leftFollowerMotor, rightMotor, rightFollowerMotor);

        leftFollowerMotor.follow(leftMotor);
        rightFollowerMotor.follow(rightMotor);
        leftMotor.stopMotor();
        rightMotor.stopMotor();

        // According to the docs, setInverted() has no effect if motor is a follower
        // Therefore only the master needs to be inverted
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        resetEncoders();
    }

    @Override
    public void periodic() {
        monitorGroup.monitorOnce();

        if(monitorGroup.getOverheatShutoff()) {
            setMotors(0, 0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(true);
        builder.setSafeState(() -> {
            setMotors(0, 0);
        });
        // Note: Not simulation-friendly
        // The call to CANSparkMax.get() will result in a buffer overflow in simulations
        builder.addDoubleProperty("Left Motor Speed", leftMotor::get, this::setLeftMotor);
        builder.addDoubleProperty("Right Motor Speed", rightMotor::get, this::setRightMotor);
    }
}
