/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TalonDriveTrain;
import frc.robot.RobotContainer;



public class ManualDrive extends CommandBase {
  private final TalonDriveTrain subsystem;

  static double rampBandLow = 0.07;
  static double rampBandHigh = 0.03;
  static boolean rampingOn = true;

  static boolean reverseDrive = false;
  static boolean precisionDrive = false;
  
  /**
   * Creates a new ManualDrive.
   */
  public ManualDrive(TalonDriveTrain subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public static boolean isRamped() {
    return rampingOn;
  }
  public static void setRamping(boolean ramping) {
      rampingOn = ramping;
  }
  public static double getRampBandHigh() {
      return rampBandHigh;
  }
  public static double getRampBandLow() {
      return rampBandLow;
  }
  public static void setRampBandHigh(double band) {
      rampBandHigh = band;
  }
  public static void setRampBandLow(double band) {
      rampBandLow = band;
  }

  public static boolean isReversed() {
      return reverseDrive;
  }
  public static boolean isPrecisionDrive() {
    return precisionDrive;
  }
  public static void setPrecisionDrive(boolean precisionDrive) {
      ManualDrive.precisionDrive = precisionDrive;
  }
  public static void togglePrecisionDrive() {
      precisionDrive = !precisionDrive;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevLeft = TalonDriveTrain.leftMotor.getMotorOutputPercent();
    prevRight = TalonDriveTrain.rightMotor.getMotorOutputPercent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = RobotContainer.driveController.getRawAxis(Constants.Drive_LEFT_RIGHT);
    double y = -RobotContainer.driveController.getRawAxis(Constants.Drive_FWD_REV);

    if(!(Math.abs(x) > Constants.DEADZONE)){
      x = 0;
      if(TalonDriveTrain.getNeutralMode() != NeutralMode.Coast) {
        TalonDriveTrain.setNeutralMode(NeutralMode.Coast);
      }
    }
    if(!(Math.abs(y) > Constants.DEADZONE)){
      if(TalonDriveTrain.getNeutralMode() != NeutralMode.Coast) {
        TalonDriveTrain.setNeutralMode(NeutralMode.Coast);
      }
      y = 0;
    }
    
    x = Math.copySign(Math.pow(x, 4), x);
    y = Math.copySign(y * y, y);

    if(TalonDriveTrain.getGear() == DriveTrain.Gear.HIGH) {
      // Half the turning rate in high gear
      x /= 3;
    }

    if(reverseDrive) {
        y = -y;
    }
    
    double l = y + x;
    double r = y - x;

    if(rampingOn) {
      double rampBand = TalonDriveTrain.getGear() == Drivetrain.Gear.HIGH ? rampBandHigh : rampBandLow;
      l = Math.max(TalonDriveTrain.getPrevLeft() - rampBand, Math.min(TalonDriveTrain.getPrevLeft() + rampBand, l));
      r = Math.max(TalonDriveTrain.getPrevRight() - rampBand, Math.min(TalonDriveTrain.getPrevRight() + rampBand, r));
    }
    if(precisionDrive) {
        l /= 2;
        r /= 2;
    }

    TalonDriveTrain.setMotors(l,r);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TalonDriveTrain.setMotors(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
