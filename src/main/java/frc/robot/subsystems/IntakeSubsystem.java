/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Motor Related Variables
    private final VictorSPX mainMotor;
    // Pneumatic Related Variables
    private final DoubleSolenoid solenoid;
    private boolean isExtended = false;

    // Motor Related
    public void setMotors(double scale) {
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

    public boolean getPistons() {
        return isExtended;
    }

    // Pneumatic related
    public void setPistons(boolean state) {
        //true for extended, false for retracted
        if (state && !isExtended) {
            isExtended = true;
            solenoid.set(DoubleSolenoid.Value.kForward);
        } else if (!state && isExtended) {
            isExtended = false;
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public IntakeSubsystem(int master, int forwardChannel, int reverseChannel) {
        // Motors
        mainMotor = new VictorSPX(master);
        // Pneumatics
        solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
        setPistons(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
