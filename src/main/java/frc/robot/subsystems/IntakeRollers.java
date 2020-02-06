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

public class IntakeRollers extends SubsystemBase {
    private final TalonSRX lowerMotor;
    private final TalonSRX upperMotor;
    private double speedMultiplier = 1.0;

    //Speed Multipliers
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = multiplier;
	}
	
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
    
    public void setMotors(double scale){
        upperMotor.set(ControlMode.PercentOutput, scale);
        lowerMotor.set(ControlMode.PercentOutput, scale);
	}

    //Neutral Mode
    NeutralMode neutralMode;
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
		upperMotor.setNeutralMode(mode);
		lowerMotor.setNeutralMode(mode);
    }
    public NeutralMode getNeutralMode() {
        return neutralMode;
    }
    
    public IntakeRollers() {
        upperMotor = new TalonSRX(Constants.UPPER_TALONSRX);
        lowerMotor = new TalonSRX(Constants.LOWER_TALONSRX);
        upperMotor.follow(lowerMotor);
        upperMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
