/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    
    private boolean overheatShutoff = false;
    private boolean overheatWarning = false;
    private boolean protectionOverridden = false;
    private BiConsumer<CANSparkMax, Double> overheatCallback;
    private BiConsumer<CANSparkMax, Double> overheatWarningCallback;
    private Runnable normalTempCallback;

    // Having a speed multiplier allows for easy adjustment of top speeds
    private double speedMultiplier = 1.0;

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
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
        leftMotor.set(overheatShutoff && !protectionOverridden ? 0 : output * speedMultiplier);
    }

    /**
     * Set the percentage output of the right motor.
     * 
     * @param output The motor output
     * @see #setMotors(double, double)
     */
    public void setRightMotor(double output) {
        rightMotor.set(overheatShutoff && !protectionOverridden ? 0 : output * speedMultiplier);
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
     * Get whether any motor is overheating.
     * 
     * <p>
     * If any motor is overheating, the setMotor methods will do nothing.
     * </p>
     * 
     * @return Whether the motors are overheating
     */
    public boolean isOverheating() {
        return overheatShutoff;
    }

    /**
     * Get whether any motor has triggered the overheat warning.
     * 
     * @return Whether the motors have triggered the warning
     */
    public boolean isOverheatWarning() {
        return overheatWarning;
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

    /**
     * Set the overheat shutoff callback.
     * 
     * <p>
     * The first argument of the callback is the motor that's overheating, and the
     * second is the temperature.
     * </p>
     * 
     * @param callback The callback
     */
    public void setOverheatShutoffCallback(BiConsumer<CANSparkMax, Double> callback) {
        overheatCallback = callback;
    }

    /**
     * Set the overheat warning callback.
     * 
     * <p>
     * The first argument of the callback is the motor that's overheating, and the
     * second is the temperature.
     * </p>
     * 
     * @param callback The callback
     */
    public void setOverheatWarningCallback(BiConsumer<CANSparkMax, Double> callback) {
        overheatWarningCallback = callback;
    }

    /**
     * Set the callback to be called when the motor's temperature has returned to
     * normal.
     * 
     * @param callback The callback
     */
    public void setNormalTempCallback(Runnable callback) {
        normalTempCallback = callback;
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

    private IdleMode idleMode;

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

        leftFollowerMotor.follow(leftMotor);
        rightFollowerMotor.follow(rightMotor);
        leftMotor.stopMotor();
        rightMotor.stopMotor();

        // According to the docs, setInverted() has no effect if motor is a follower
        // Therefore only the master needs to be inverted
        leftMotor.setInverted(true);

        leftEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        resetEncoders();
    }

    @Override
    public void periodic() {
        CANSparkMax[] motors = { leftMotor, leftFollowerMotor, rightMotor, rightFollowerMotor };
        boolean shutoff = false;
        boolean warning = false;
        for (var motor : motors) {
            // Check every motor for temperature
            double temp = motor.getMotorTemperature();

            if (temp >= Constants.MOTOR_SHUTOFF_TEMP) {
                // Only call the callback if not already overheating
                if (!overheatShutoff && overheatCallback != null) {
                    overheatCallback.accept(motor, temp);
                }
                shutoff = true;
            } else if (temp >= Constants.MOTOR_WARNING_TEMP) {
                if (!overheatWarning && overheatWarningCallback != null) {
                    overheatWarningCallback.accept(motor, temp);
                }
                warning = true;
            }
        }
        // Run the return to normal callback
        if ((overheatShutoff || overheatWarning) && (!shutoff && !warning) && normalTempCallback != null) {
            normalTempCallback.run();
        }

        overheatShutoff = shutoff;
        overheatWarning = warning;
    }

}
