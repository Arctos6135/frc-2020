/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class TalonDriveTrain extends SubsystemBase {

	double leftLastRate = 0;
	double rightLastRate = 0;
    double lastTime;

    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;
    private final Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
    private final Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k4X);
    
    // speed multiplier allows for easy adjustment of top speeds
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
     * @param left The left side motors percent output
     * @param right The right side motors percent output
     */
    public void setMotors(double left, double right){
        setLeftMotor(-left);
        setRightMotor(right);
    }

    public void setLeftMotor(double output){
		leftMotor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, output * speedMultiplier)));
	}
	
	public void setRightMotor(double output){
		rightMotor.set(ControlMode.PercentOutput, Math.max(-1, Math.min(1, output * speedMultiplier)));
	}
    
    // Encoders
	/** 
	 * Reset the left and right encoders
	*/
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	/**
	 * Gets the distance travelled by the left encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the left encoder
	 */
	public double getLeftDistance() {
		return leftEncoder.getDistance();
	}

	/**
	 * Gets the distance travelled by the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getDistance() getDistance()} method of the {@code Encoder} class
	 * @return The distance travelled by the right encoder
	 */
	public double getRightDistance() {
		return rightEncoder.getDistance();
	}

	/**
	 * Gets the speed reading of the left encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the left encoder
	 */
	public double getLeftSpeed() {
		return leftEncoder.getRate();
	}

	/**
	 * Gets the speed reading of the right encoder, using the
	 * {@link edu.wpi.first.wpilibj.Encoder#getRate() getRate()} method of the {@code Encoder} class
	 * @return The speed of the right encoder
	 */
	public double getRightSpeed() {
		return rightEncoder.getRate();
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
    	double leftRate = leftEncoder.getRate();
    	double rightRate = rightEncoder.getRate();
    	double leftAccel = (leftRate - leftLastRate) / dt;
    	double rightAccel = (rightRate - rightLastRate) / dt;
    	leftLastRate = leftRate;
    	rightLastRate = rightRate;
    	lastTime = Timer.getFPGATimestamp();
    	return new double[] { leftAccel, rightAccel };
	}
	
	NeutralMode neutralMode;
    /**
     * Sets the neutral mode (brake or coast) of all the drivetrain motors.
     */
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
        
		leftMotor.setNeutralMode(mode);
		rightMotor.setNeutralMode(mode);
    }

    /**
     * Gets neutral mode of all drivetrain motors.
     */
    public NeutralMode getNeutralMode() {
        return neutralMode;
	}

    /**
     * Creates a new DriveTrain.
     */
    public TalonDriveTrain() {
		leftMotor = new TalonSRX(Constants.LEFT_TALONSRX);
		rightMotor = new TalonSRX(Constants.LEFT_TALONSRX);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


}
