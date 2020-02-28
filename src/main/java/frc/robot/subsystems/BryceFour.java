/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BryceFour extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public void setMotorSpeed(double motorSpeed) {
        rightMotor.set(motorSpeed);
    }

	/**
	* Creates a new BryceFour.
	*/
  	public BryceFour(int leftMotor, int rightMotor) {
        this.leftMotor = new CANSparkMax(leftMotor, MotorType.kBrushless);
        this.rightMotor = new CANSparkMax(rightMotor, MotorType.kBrushless);

        this.leftMotor.follow(this.rightMotor);
        // limit to 20A
        this.rightMotor.setSmartCurrentLimit(20);
  	}

  	@Override
  	public void periodic() {
		// This method will be called once per scheduler run
  	}
}
