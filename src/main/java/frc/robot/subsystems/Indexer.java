/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    //motors
    private final TalonSRX leftRoller;
	private final TalonSRX rightRoller;
	
	// other variables
	private double motorSpeed;

	public void setMotorSpeed(double motorSpeed) {
		this.motorSpeed = motorSpeed;
	}

	public double getMotorSpeed() {
		return motorSpeed;
	}

    public void startMotors() {
		leftRoller.set(ControlMode.PercentOutput, motorSpeed);
	}

	public void stopMotors() {
		leftRoller.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * Creates a new Indexer.
	 */
	public Indexer(int leftMotor, int rightMotor) {
        leftRoller = new TalonSRX(leftMotor);
        rightRoller = new TalonSRX(rightMotor);
		rightRoller.follow(leftRoller);

		motorSpeed =  0.5;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
