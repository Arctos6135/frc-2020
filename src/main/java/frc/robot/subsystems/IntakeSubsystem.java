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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //Motor Related Variables
    private final TalonSRX mainMotor;
    //Pneumatic Related Variables
    private final DoubleSolenoid solenoid;
    private boolean isExtended = false;

    //Motor Related
    public void setMotors(double scale){
        mainMotor.set(ControlMode.PercentOutput, scale);
	}

    // | Neutral Mode
    NeutralMode neutralMode;
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
		mainMotor.setNeutralMode(mode);
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    public boolean getExtended() {
        return isExtended;
    }

    //Pneumatic related
    public void setPistons(int Extend){
        //Set Extended to either 0 or 1
        if(Extend==1&!isExtended){
            isExtended = true;
            solenoid.set(DoubleSolenoid.Value.kForward);
        }
        else if(Extend==0&isExtended){
            isExtended = false;
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
        
    }

    public IntakeSubsystem(int master, int forwardChannel, int reverseChannel) {
        //Motors
        mainMotor = new TalonSRX(master);
        //Pneumatics
        solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
        setPistons(1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
