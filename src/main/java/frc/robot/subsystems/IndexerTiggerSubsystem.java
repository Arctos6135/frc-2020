/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IndexerTiggerSubsystem extends SubsystemBase {

    // Tigger related variables
    private final TalonSRX backRoller;
    private final TalonSRX frontRoller;
    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;

    private int powercellCount;
    private double tiggerMotorSpeed;
    
    // Indexer related variables
    private final TalonSRX leftRoller;
    private final TalonSRX rightRoller;
    
    private double indexerMotorSpeed;

    public void addPowercellCount() {
        powercellCount += 1;
    }

    public void reducePowercellCount() {
        powercellCount -= 1;
    }

    public int getPowercellCount()  {
        return powercellCount;
    }

    public void clearPowercellCount() {
        powercellCount = 0;
    }
    
    public void setTiggerMotorSpeed(double tiggerMotorSpeed) {
        this.tiggerMotorSpeed = tiggerMotorSpeed;
    }

    public double getTiggerMotorSpeed() {
        return tiggerMotorSpeed;
    }

    public void startBackRoller() {
        backRoller.set(ControlMode.PercentOutput, tiggerMotorSpeed);
    }

    public void stopBackRoller() {
        backRoller.set(ControlMode.PercentOutput, 0);
    }

    public void startFrontRoller() {
        frontRoller.set(ControlMode.PercentOutput, tiggerMotorSpeed);
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

    public void setIndexerMotorSpeed(double indexerMotorSpeed) {
		this.indexerMotorSpeed = indexerMotorSpeed;
	}

	public double getIndexerMotorSpeed() {
		return indexerMotorSpeed;
	}

    public void startIndexer() {
		leftRoller.set(ControlMode.PercentOutput, indexerMotorSpeed);
	}

	public void stopIndexer() {
		leftRoller.set(ControlMode.PercentOutput, 0);
	}

    /**
      * Creates a new IndexerTiggerSubsystem.
      */
    public IndexerTiggerSubsystem(int backMotor, int frontMotor, int bottomSensorChannel, int topSensorChannel,int leftMotor, int rightMotor) {
        // Tigger related setup
        backRoller = new TalonSRX(backMotor);
        frontRoller = new TalonSRX(frontMotor);
        bottomSensor = new DigitalInput(bottomSensorChannel);
        topSensor = new DigitalInput(topSensorChannel);
        
        tiggerMotorSpeed = 0.5;

        // Indexer related setup
        leftRoller = new TalonSRX(leftMotor);
        rightRoller = new TalonSRX(rightMotor);
		rightRoller.follow(leftRoller);

		indexerMotorSpeed =  0.5;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
