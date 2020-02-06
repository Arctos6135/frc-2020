/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private final DoubleSolenoid solenoid;
    
	public Pneumatics(int forwardChannel, int reverseChannel) {
		solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
        set(DoubleSolenoid.Value.kReverse); //Start raised
	}

	public void set(DoubleSolenoid.Value value) {
        solenoid.set(value);
	}
	
	@Override
	public void periodic() {
    	// This method will be called once per scheduler run
  	}
}
