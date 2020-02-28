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

/**
 * The intake subsystem.
 * 
 * @author Jeremy Xie
 */
public class IntakeSubsystem extends SubsystemBase {
    // Motor Related Variables
    private final VictorSPX mainMotor;
    // Pneumatic Related Variables
    private final DoubleSolenoid solenoid;
    private boolean isExtended = false;

    /**
     * Set the speed of the motor.
     * 
     * @param scale The motor speed
     */
    public void setMotor(double scale) {
        mainMotor.set(ControlMode.PercentOutput, scale);
    }

    // | Neutral Mode
    NeutralMode neutralMode;

    /**
     * Set the neutral mode of the intake.
     * 
     * @param mode The neutral mode
     */
    public void setNeutralMode(NeutralMode mode) {
        neutralMode = mode;
        mainMotor.setNeutralMode(mode);
    }

    /**
     * Get the neutral mode of the intake.
     * 
     * @return The neutral mode
     */
    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    /**
     * Get whether the intake is extended.
     * 
     * @return Whether the intake is extended
     */
    public boolean isExtended() {
        return isExtended;
    }

    /**
     * Set whether the intake should be extended.
     * 
     * @param state Whether the intake should be extended
     */
    public void setExtended(boolean state) {
        // true for extended, false for retracted
        if (state && !isExtended) {
            isExtended = true;
            solenoid.set(DoubleSolenoid.Value.kForward);
        } else if (!state && isExtended) {
            isExtended = false;
            solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Create a new intake subsystem.
     * 
     * @param roller         The CAN ID of the roller motor controller
     * @param forwardChannel The double solenoid's forward channel
     * @param reverseChannel The double solenoid's reverse channel
     */
    public IntakeSubsystem(int roller, int forwardChannel, int reverseChannel) {
        // Motors
        mainMotor = new VictorSPX(roller);
        // Pneumatics
        solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
        setExtended(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
