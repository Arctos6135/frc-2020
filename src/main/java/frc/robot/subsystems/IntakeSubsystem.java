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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //Motor Related Variables
    private final TalonSRX lowerMotor;
    private final TalonSRX upperMotor;
    //Pneumatic Related Variables
    private final int forwardChannel = 1;
    private final int reverseChannel = 2;
    private final DoubleSolenoid solenoid;

    //Motor Related
    public void setMotors(double scale){
        upperMotor.set(ControlMode.PercentOutput, scale);
        lowerMotor.set(ControlMode.PercentOutput, scale);
	}

    // | Neutral Mode
    NeutralMode neutralMode;
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
		upperMotor.setNeutralMode(mode);
		lowerMotor.setNeutralMode(mode);
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    //Pneumatic related
    public void setPistons(DoubleSolenoid.Value value){
        solenoid.set(value);
    }

    public IntakeSubsystem() {
        //Motors
        upperMotor = new TalonSRX(Constants.UPPER_ROLLER_TALONSRX);
        lowerMotor = new TalonSRX(Constants.LOWER_ROLLER_TALONSRX);
        upperMotor.follow(lowerMotor);
        upperMotor.setInverted(true);
        //Pneumatics
        solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
        setPistons(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
