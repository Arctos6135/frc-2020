/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BryceFour;

public class Elevator extends CommandBase {

    private final BryceFour bryceFour;
    private final XboxController controller;
    
	private static boolean override;
    private final double DEAD_ZONE = 0.15;

	/**
	* Creates a new Elevator.
	*/
  	public Elevator(BryceFour bryceFour, XboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.bryceFour = bryceFour;
		this.controller = controller;

        addRequirements(bryceFour);
 	}

     public static void toggleOverride() {
        if (override) {
            override = false;
        }
        else {
            override = true;
        }
     }

  	// Called when the command is initially scheduled.
  	@Override
  	public void initialize() {
  	}

  	// Called every time the scheduler runs while the command is scheduled.
  	@Override
  	public void execute() {
        double yValue = controller.getRawAxis(Constants.BRYCE_FOUR_ELEVATOR_CONTROL);
		
		TeleopDrive.applyDeadband(yValue, DEAD_ZONE);

		// Will only raise if remaining match time is less than 30
		// or is in override mode.
		if (!override && DriverStation.getInstance().getMatchTime() <= 30) {
			bryceFour.setMotorSpeed(yValue);
		}
		else if (!override && yValue < 0) {
			bryceFour.setMotorSpeed(yValue);
		}
		else if (override){
			bryceFour.setMotorSpeed(yValue);
		}

  	}

  	// Called once the command ends or is interrupted.
  	@Override
  	public void end(boolean interrupted) {
		  	bryceFour.setMotorSpeed(0);
  	}

  	// Returns true when the command should end.
  	@Override
  	public boolean isFinished() {
		return false;
  	}
}
