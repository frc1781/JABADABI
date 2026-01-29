package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    //REAL MOTOR
    private SparkMax motorLeft;
    private SparkMax motorRight;

    private RelativeEncoder motorLeftEncoder;
    private RelativeEncoder motorRightEncoder;

    private PIDController pidController;
    private SparkClosedLoopController motorController;

    public Climber() {
        motorLeft = new SparkMax(Constants.Climber.MOTOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        motorRight = new SparkMax(Constants.Climber.MOTOR_RIGHT, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig motorConfigLeft = new SparkMaxConfig();
        motorConfigLeft.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigLeft.inverted(true); //idfk the inversion
        motorConfigLeft.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigLeft.smartCurrentLimit(30);
        motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motorConfigRight = new SparkMaxConfig();
        motorConfigRight.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigRight.follow(Constants.Climber.MOTOR_LEFT, true); //idfk the inversion
        motorConfigRight.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigRight.smartCurrentLimit(30);
        motorRight.configure(motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorLeftEncoder = motorLeft.getEncoder();
        motorRightEncoder = motorRight.getEncoder();

        motorController = motorLeft.getClosedLoopController();
        pidController = new PIDController(Constants.Climber.P, Constants.Climber.I, Constants.Climber.D);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/ClimberSetpoint", motorController.getSetpoint());
        Logger.recordOutput(getName() + "/ClimberPositionLeft", motorLeftEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleLeft", motorLeft.getAppliedOutput());
        Logger.recordOutput(getName() + "/ClimberPositionRight", motorRightEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleRight", motorRight.getAppliedOutput());
    }

    public Command ascend() {
        return new InstantCommand(() -> {
            motorController.setSetpoint(0, ControlType.kPosition);
            // motorLeft.set(pidController.calculate(motorLeftEncoder.getPosition(), 0));
        }, this).repeatedly();
    }

    public Command descend() {
        return new InstantCommand(() -> {
            motorController.setSetpoint(4, ControlType.kPosition);
            // motorLeft.set(pidController.calculate(motorLeftEncoder.getPosition(), 4));
        }, this).repeatedly();
    }
}
