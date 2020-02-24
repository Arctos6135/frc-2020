/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Indexer-Tigger subsystem is responsible for storing and delivering the
 * Power Cells from the Intake to the Shooter.
 */
public class IndexerTiggerSubsystem extends SubsystemBase {

    // Tigger related variables
    private final VictorSPX backRoller;
    private final VictorSPX frontRoller;
    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;

    private int powercellCount;
    private double tiggerMotorSpeed;

    // Indexer related variables
    private final VictorSPX leftRoller;
    private final VictorSPX rightRoller;

    private double indexerMotorSpeed;

    public void addPowercellCount() {
        powercellCount += 1;
    }

    public void reducePowercellCount() {
        powercellCount -= 1;
    }

    public int getPowercellCount() {
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

    public void startIndexerSameDirection() {
        leftRoller.set(ControlMode.PercentOutput, indexerMotorSpeed);
        rightRoller.set(ControlMode.PercentOutput, indexerMotorSpeed);
    }

    public void startIndexerOppositeDirections() {
        leftRoller.set(ControlMode.PercentOutput, indexerMotorSpeed);
        rightRoller.set(ControlMode.PercentOutput, -indexerMotorSpeed);
    }

    public void stopIndexer() {
        leftRoller.set(ControlMode.PercentOutput, 0);
        rightRoller.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Creates a new IndexerTiggerSubsystem.
     */
    public IndexerTiggerSubsystem(int backMotor, int frontMotor, int bottomSensorChannel, int topSensorChannel,
            int leftMotor, int rightMotor) {
        // Tigger related setup
        backRoller = new VictorSPX(backMotor);
        frontRoller = new VictorSPX(frontMotor);
        bottomSensor = new DigitalInput(bottomSensorChannel);
        topSensor = new DigitalInput(topSensorChannel);

        tiggerMotorSpeed = 0.5;

        // Indexer related setup
        leftRoller = new VictorSPX(leftMotor);
        rightRoller = new VictorSPX(rightMotor);
        rightRoller.setInverted(true);

        indexerMotorSpeed = 0.5;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
