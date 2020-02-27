/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BryceFour extends SubsystemBase {

    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;

    public void setMotorSpeed(double motorSpeed) {
        rightMotor.set(ControlMode.PercentOutput, motorSpeed);
    }

	/**
	* Creates a new BryceFour.
	*/
  	public BryceFour(int leftMotor, int rightMotor) {
        this.leftMotor = new TalonSRX(leftMotor);
        this.rightMotor = new TalonSRX(rightMotor);
        this.leftMotor.follow(this.rightMotor);
        // Enable the current limit
        // Once current exceeds 40A for 1.0s, limit current to 20A.
        this.rightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 40, 1));
  	}

  	@Override
  	public void periodic() {
		// This method will be called once per scheduler run
  	}
}
