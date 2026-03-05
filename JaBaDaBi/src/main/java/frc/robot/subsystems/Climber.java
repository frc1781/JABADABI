package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robots;

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
    private SparkMax motorRight;

    private RelativeEncoder motorLeftEncoder;
    private RelativeEncoder motorRightEncoder;

    private SparkClosedLoopController motorLeftController;
    private SparkClosedLoopController motorRightController;

    private boolean atPosition;

    public Climber() {
        if (Robot.getRobot() != Robots.SAVITAR) {
            return;
        }  
        motorLeft = new SparkMax(Constants.Climber.MOTOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        motorRight = new SparkMax(Constants.Climber.MOTOR_RIGHT, SparkLowLevel.MotorType.kBrushless);
        atPosition = false;

        SparkMaxConfig motorConfigLeft = new SparkMaxConfig();
        motorConfigLeft.idleMode(SparkBaseConfig.IdleMode.kCoast);
        motorConfigLeft.inverted(false); // idfk the inversion
        motorConfigLeft.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigLeft.closedLoop.feedForward.apply(Constants.Climber.FEED_FORWARD_CONFIG);
        motorConfigLeft.smartCurrentLimit(40);
        motorConfigLeft.softLimit.forwardSoftLimit(140 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.softLimit.reverseSoftLimit(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.encoder.velocityConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION_PER_SECOND);
        motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motorConfigRight = new SparkMaxConfig();
        motorConfigRight.idleMode(SparkBaseConfig.IdleMode.kCoast);
        motorConfigRight.inverted(true);
        motorConfigRight.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorConfigRight.closedLoop.feedForward.apply(Constants.Climber.FEED_FORWARD_CONFIG);
        motorConfigRight.smartCurrentLimit(40);
        motorConfigRight.softLimit.forwardSoftLimit(135 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigRight.softLimit.reverseSoftLimit(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigRight.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigRight.encoder.velocityConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION_PER_SECOND);
        motorRight.configure(motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        motorLeftEncoder = motorLeft.getEncoder();
        motorRightEncoder = motorRight.getEncoder();
        motorLeftEncoder.setPosition(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorRightEncoder.setPosition(0 * Constants.Climber.INCHES_PER_REVOLUTION);
        motorLeftController = motorLeft.getClosedLoopController();
        motorRightController = motorRight.getClosedLoopController();
    }

    @Override
    public void periodic() {
        if (Robot.getRobot() != Robots.SAVITAR) {
            return;
        }  
        Logger.recordOutput(getName() + "/ClimberPositionLeft", motorLeftEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleLeft", motorLeft.getAppliedOutput());
        Logger.recordOutput(getName() + "/ClimberPositionRight", motorRightEncoder.getPosition());
        Logger.recordOutput(getName() + "/ClimberDutyCycleRight", motorRight.getAppliedOutput());
        Logger.recordOutput(getName() + "/ClimberSetPointRight", motorRightController.getSetpoint());
        Logger.recordOutput(getName() + "/ClimberSetPointLeft", motorLeftController.getSetpoint());
    }

    public boolean atPosition() {
        if (Robot.getRobot() != Robots.SAVITAR) {
            return true;
        }  
        if (atPosition) {
            return true;
        }
        atPosition = motorLeft.getEncoder().getPosition() >= 15 && motorRight.getEncoder().getPosition() >= 15;
        return atPosition;
    }

    public Command ascend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(-45, ControlType.kVelocity);
            motorRightController.setSetpoint(-45, ControlType.kVelocity);
           Logger.recordOutput(getName() + "/currentCommand", "ascend");
        }, this).repeatedly();
    }

    public Command descend() {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(45, ControlType.kVelocity);
            motorRightController.setSetpoint(45, ControlType.kVelocity);
         Logger.recordOutput(getName() + "/currentCommand", "descend");
        }, this).repeatedly();
    }

    public Command setClimber(DoubleSupplier setPoint) {
        return new InstantCommand(() -> {
            motorLeftController.setSetpoint(setPoint.getAsDouble(), ControlType.kPosition);
            motorRightController.setSetpoint(setPoint.getAsDouble(), ControlType.kPosition);
        }, this).until(() -> atPosition);
    }

    public Command idle() {
        return new RunCommand(() -> {
            motorLeftController.setSetpoint(0, ControlType.kVelocity);
            motorRightController.setSetpoint(0, ControlType.kVelocity);
            Logger.recordOutput(getName() + "/currentCommand", "idle");
        }, this);
    }

    public RelativeEncoder getLeftEncoder() {
        return motorLeftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return motorRightEncoder;
    }
}
