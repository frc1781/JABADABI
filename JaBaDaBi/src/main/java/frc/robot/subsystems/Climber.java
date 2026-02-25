package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    private SparkMax motorRight;

    private RelativeEncoder motorLeftEncoder;
    private RelativeEncoder motorRightEncoder;

    private SparkClosedLoopController motorLeftController;
    private SparkClosedLoopController motorRightController;

    public Climber() {
        motorLeft = new SparkMax(Constants.Climber.MOTOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        motorRight = new SparkMax(Constants.Climber.MOTOR_RIGHT, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig motorConfigLeft = new SparkMaxConfig();
        motorConfigLeft.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigLeft.inverted(true); // idfk the inversion
        motorConfigLeft.encoder.velocityConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigLeft.smartCurrentLimit(40);
        motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motorConfigRight = new SparkMaxConfig();
        motorConfigRight.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigRight.inverted(true);
        motorConfigRight.encoder.velocityConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigRight.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigRight.smartCurrentLimit(40);
        motorRight.configure(motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorLeftEncoder = motorLeft.getEncoder();
        motorRightEncoder = motorRight.getEncoder();

        motorLeftController = motorLeft.getClosedLoopController();
        motorRightController = motorRight.getClosedLoopController();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/ClimberPositionLeft", motorLeftEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleLeft", motorLeft.getAppliedOutput());
        Logger.recordOutput(getName() + "/ClimberPositionRight", motorRightEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleRight", motorRight.getAppliedOutput());
    }

    public Command ascend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(-1, ControlType.kVelocity);
            motorRightController.setSetpoint(-1, ControlType.kVelocity);
        }, this).repeatedly();
    }

    public Command descend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(1, ControlType.kVelocity);
            motorRightController.setSetpoint(1, ControlType.kVelocity);
        }, this).repeatedly();
    }

    public Command idle() {
        return new RunCommand(() -> {
            motorLeftController.setSetpoint(0, ControlType.kVelocity);
            motorRightController.setSetpoint(0, ControlType.kVelocity);
        }, this);
    }

    public RelativeEncoder getLeftEncoder() {
        return motorLeftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return motorRightEncoder;
    }
}
