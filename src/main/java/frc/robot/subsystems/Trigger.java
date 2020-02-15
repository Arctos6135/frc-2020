/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Trigger extends SubsystemBase {

    // motors
    private final TalonSRX backRoller;
    private final TalonSRX frontRoller;
    // sensors
    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;
    
    // other variables
    private int powercellCount;
    private double backMotorSpeed;
    private double frontMotorSpeed;
    
    public void addPowercellCount() {
        powercellCount += 1;
    }

    public int getPowercellCount()  {
        return powercellCount;
    }

    public void clearPowercellCount() {
        powercellCount = 0;
    }
    
    public void setBackMotorSpeed(double backMotorSpeed) {
        this.backMotorSpeed = backMotorSpeed;
    }

    public double getBackMotorSpeed() {
        return backMotorSpeed;
    }

    public void setFrontMotorSpeed(double frontMotorSpeed) {
        this.frontMotorSpeed = frontMotorSpeed;
    }

    public double getFrontMotorSpeed() {
        return frontMotorSpeed;
    }

    public void startBackRoller() {
        backRoller.set(ControlMode.PercentOutput, backMotorSpeed);
    }

    public void stopBackRoller() {
        backRoller.set(ControlMode.PercentOutput, 0);
    }

    public void startFrontRoller() {
        frontRoller.set(ControlMode.PercentOutput, frontMotorSpeed);
    }

    public void stopFrontRoller() {
        frontRoller.set(ControlMode.PercentOutput, 0);
    }

    public boolean isTopBlocked() {
        return topSensor.get();
    }

    public boolean isBottomBlocked() {
        return bottomSensor.get();
    }

    /**
      * Creates a new Trigger.
      */
    public Trigger() {
        // motors and sensors
        backRoller = new TalonSRX(Constants.TRIGGER_BACK_ROLLER);
        frontRoller = new TalonSRX(Constants.TRIGGER_FRONT_ROLLER);
        bottomSensor = new DigitalInput(Constants.TRIGGER_BOTTON_SENSOR);
        topSensor = new DigitalInput(Constants.TRIGGER_TOP_SENSOR);
        
        frontMotorSpeed = 0.5;
        backMotorSpeed = 0.5;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
