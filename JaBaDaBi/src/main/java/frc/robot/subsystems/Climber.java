package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {

    // REAL MOTOR
    private SparkMax motorLeft;
    // private SparkMax motorRight;

    private RelativeEncoder motorLeftEncoder;
    // private RelativeEncoder motorRightEncoder;

    private SparkClosedLoopController motorLeftController;
    // private SparkClosedLoopController motorRightController;

    private boolean belowPosition, abovePosition;
    private double desiredPosition;

    public Climber() {
        motorLeft = new SparkMax(Constants.Climber.MOTOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        // motorRight = new SparkMax(Constants.Climber.MOTOR_RIGHT, SparkLowLevel.MotorType.kBrushless);
        belowPosition = false;
        abovePosition = false;
        desiredPosition = 6.7;  //something

        SparkMaxConfig motorConfigLeft = new SparkMaxConfig();
        motorConfigLeft.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigLeft.inverted(false); // idfk the inversion
        motorConfigLeft.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigLeft.closedLoop.feedForward.apply(Constants.Climber.FEED_FORWARD_CONFIG);
        motorConfigLeft.smartCurrentLimit(40);
        motorConfigLeft.softLimit.forwardSoftLimit(135 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.softLimit.reverseSoftLimit(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.softLimit.forwardSoftLimitEnabled(true);
        motorConfigLeft.softLimit.reverseSoftLimitEnabled(true);
        motorConfigLeft.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.encoder.velocityConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION_PER_SECOND);
        motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        motorLeftEncoder = motorLeft.getEncoder();
        // motorRightEncoder = motorRight.getEncoder();
        motorLeftEncoder.setPosition(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        // motorRightEncoder.setPosition(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorLeftController = motorLeft.getClosedLoopController();
        // motorRightController = motorRight.getClosedLoopController();
        Logger.recordOutput(getName() + "/currentCommand", "noneYet");
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/ClimberPositionLeft", motorLeftEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleLeft", motorLeft.getAppliedOutput());
        // Logger.recordOutput(getName() + "/ClimberPositionRight", motorRightEncoder.getPosition());
        // Logger.recordOutput(getName() + "/ClimberDutyCycleRight", motorRight.getAppliedOutput());
        // Logger.recordOutput(getName() + "/ClimberSetPointRight", motorRightController.getSetpoint());
        Logger.recordOutput(getName() + "/ClimberSetPointLeft", motorLeftController.getSetpoint());
        Logger.recordOutput(getName() + "/desiredPosition", desiredPosition);
        Logger.recordOutput(getName() + "/belowPosition", belowPosition);
        Logger.recordOutput(getName() + "/abovePosition", abovePosition);
    }

    private boolean determineBelowPosition() {
        belowPosition = motorLeftEncoder.getPosition() <= desiredPosition;
        return belowPosition;
    }
    private boolean determineAbovePosition() {
        abovePosition = motorLeftEncoder.getPosition() >= desiredPosition;
        return abovePosition;
    }

    public Command ascend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(-45, ControlType.kVelocity);
            // motorRightController.setSetpoint(-45, ControlType.kVelocity);
           Logger.recordOutput(getName() + "/currentCommand", "ascend");
        }, this).repeatedly();
    }

    public Command descend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(45, ControlType.kVelocity);
            // motorRightController.setSetpoint(45, ControlType.kVelocity);
         Logger.recordOutput(getName() + "/currentCommand", "descend");
        }, this).repeatedly();
    }

    public Command resetClimber() {
        return new InstantCommand(() -> {
            motorLeft.getEncoder().setPosition(0);
            // motorRight.getEncoder().setPosition(0);
        });
    }

    public Command raiseClimber(DoubleSupplier setPoint) {
        return new FunctionalCommand(
            () -> { 
                abovePosition = false; 
                desiredPosition = setPoint.getAsDouble();
                Logger.recordOutput(getName() + "/currentCommand", "raiseClimber");
            },
            () -> {
                motorLeftController.setSetpoint(85, ControlType.kVelocity);
                // motorRightController.setSetpoint(85, ControlType.kVelocity);
            },
            (interrupted) -> {
                motorLeftController.setSetpoint(0, ControlType.kVelocity);
                // motorRightController.setSetpoint(0, ControlType.kVelocity);
                Logger.recordOutput(getName() + "/currentCommand", "none");
            },
            () -> determineAbovePosition(),
            this);
    };

    public Command lowerClimber(DoubleSupplier setPoint) {
        return new FunctionalCommand(
            () -> { 
                belowPosition = false; 
                desiredPosition = setPoint.getAsDouble();
                Logger.recordOutput(getName() + "/currentCommand", "lowerClimber");
            },
            () -> {
                motorLeftController.setSetpoint(-45, ControlType.kVelocity);
                // motorRightController.setSetpoint(-45, ControlType.kVelocity);
            },
            (interrupted) -> {
                motorLeftController.setSetpoint(0, ControlType.kVelocity);
                // motorRightController.setSetpoint(0, ControlType.kVelocity);
                Logger.recordOutput(getName() + "/currentCommand", "none");
            },
            () -> determineBelowPosition(),
            this);
    };


    public Command idle() {
        return new RunCommand(() -> {
            motorLeftController.setSetpoint(0, ControlType.kVelocity);
            // motorRightController.setSetpoint(0, ControlType.kVelocity);
            Logger.recordOutput(getName() + "/currentCommand", "idle");
        }, this);
    }

    public RelativeEncoder getLeftEncoder() {
        return motorLeftEncoder;
    }

    // public RelativeEncoder getRightEncoder() {
    //     return motorRightEncoder;
    // }
}
