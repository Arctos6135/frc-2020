/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {
    
    double leftLastRate = 0, rightLastRate = 0;
    double lastTime;

    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightFollowerMotor;
    private final CANSparkMax leftFollowerMotor;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;
    
    // Having a speed multiplier allows for easy adjustment of top speeds
    private double speedMultiplier = 1.0;

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
    }
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    /**
     * Sets the left and right side motors of the drivetrain. 
     * The input values are first multiplied by the speed multiplier (see {@link #setSpeedMultiplier(double)}), 
     * and then constrained to [-1, 1].
     * @param left The left side motors percent output (inverted)
     * @param right The right side motors percent output
     */
    public void setMotors(double left, double right){
        // Inverted in constructor
        setLeftMotor(left);
        setRightMotor(right);
    }

    public void setLeftMotor(double output){
        leftMotor.set(Math.max(-1, Math.min(1, output * speedMultiplier)));
    }

    public void setRightMotor(double output){
        rightMotor.set(Math.max(-1, Math.min(1, output * speedMultiplier)));
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
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the left encoder
	 */
	public double getLeftDistance() {
		return leftEncoder.getPosition();
	}

	/**
	 * Gets the distance travelled by the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the right encoder
	 */
	public double getRightDistance() {
		return rightEncoder.getPosition();
	}

	/**
	 * Gets the speed reading of the left encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the left encoder
	 */
	public double getLeftSpeed() {
		return leftEncoder.getVelocity();
	}

	/**
	 * Gets the speed reading of the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the right encoder
	 */
	public double getRightSpeed() {
		return rightEncoder.getVelocity();
	}

	/**
     * Calculates the acceleration of both sides.<br>
     * <br>
     * Instead of being read directly from the encoder, the acceleration is calculated by deriving the result
     * of {@link #getLeftSpeed()} and {@link #getRightSpeed()}. The derivation is done only when this method
     * is called, therefore the result will be more accurate if this method was called a short time ago.
     * However, please note that two subsequent calls right after each other may yield a result of 0, as the
     * last known encoder rate from {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} may not have time
     * to update.
     * @return An array containing the acceleration of both sides, with element 0 being the left and element 1 
     * being the right
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
    

    IdleMode idleMode;
    /**
     * Sets the IDLEmode (brake or coast) of all the drivetrain motors.
     */
    public void setMotorMode (IdleMode mode){
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
     * Creates a new drivetrain.
     */
    public DriveTrain() {
        rightMotor = new CANSparkMax(Constants.RIGHT_CANSPARKMAX, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.LEFT_CANSPARKMAX, MotorType.kBrushless);
        rightFollowerMotor = new CANSparkMax(Constants.RIGHT_CANSPARKMAX_FOLLOWER, MotorType.kBrushless);
        leftFollowerMotor = new  CANSparkMax(Constants.LEFT_CANSPARKMAX_FOLLOWER, MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, Constants.COUNT_PER_REVOLUTION);
        leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, Constants.COUNT_PER_REVOLUTION);

        leftFollowerMotor.follow(leftMotor);
        rightFollowerMotor.follow(rightMotor);
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        leftMotor.setInverted(true);

        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        leftEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(Constants.POSITION_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
