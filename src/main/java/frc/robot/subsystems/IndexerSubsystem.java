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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonSRX leftMotor;
    private final TalonSRX rightMotor;
    
    public void setMotors(double scale){
        leftMotor.set(ControlMode.PercentOutput, scale);
        rightMotor.set(ControlMode.PercentOutput, scale);
	}
    
    NeutralMode neutralMode;
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
		leftMotor.setNeutralMode(mode);
		rightMotor.setNeutralMode(mode);
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    public IndexerSubsystem() {
        leftMotor = new TalonSRX(Constants.UPPER_ROLLER_TALONSRX);
        rightMotor = new TalonSRX(Constants.LOWER_ROLLER_TALONSRX);
        rightMotor.follow(leftMotor);
        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
