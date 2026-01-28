package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
    private SparkMax motorLeft;
    private SparkMax motorRight;

    private SparkClosedLoopController motorController;

    public Climber() {
        motorLeft = new SparkMax(Constants.Climber.MOTOR_LEFT, SparkLowLevel.MotorType.kBrushless);
        motorRight = new SparkMax(Constants.Climber.MOTOR_RIGHT, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig motorConfigLeft = new SparkMaxConfig();
        motorConfigLeft.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motorConfigLeft.inverted(true); //idfk
        motorConfigLeft.encoder.positionConversionFactor(Constants.Climber.INCHES_PER_REVOLUTION);
        motorConfigLeft.closedLoop.apply(Constants.Climber.CLOSED_LOOP_CONFIG);
        motorLeft.configure(motorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig motorConfigRight = new SparkMaxConfig();
        motorConfigRight.follow(Constants.Climber.MOTOR_LEFT, true); //idfk
        motorRight.configure(motorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorController = motorLeft.getClosedLoopController();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/ClimberPosition", motorController.getSetpoint());
    }

    public Command ascend(){
        return new InstantCommand(() -> {
            motorController.setSetpoint(motorController.getSetpoint()  + 1, ControlType.kPosition);
        }, this).repeatedly();
    }

    public Command descend(){
        return new InstantCommand(() -> {
            motorController.setSetpoint(motorController.getSetpoint()  - 1, ControlType.kPosition);
        }, this).repeatedly();
    }

}
