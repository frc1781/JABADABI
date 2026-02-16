package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {

  private SparkMax deployMotor;
  private SparkMax intakeMotor;

  private SparkMaxConfig deployMotorConfig;
  private SparkMaxConfig intakeMotorConfig;
  private SparkClosedLoopController iCollectorM;
  private SparkClosedLoopController dCollectorM;
  private double iMotorPower;
  private double dMotorPower;

  public Collector() {

    deployMotor = new SparkMax(Constants.Collector.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    intakeMotor = new SparkMax(Constants.Collector.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(40);
    deployMotorConfig.inverted(false);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.follow(Constants.Collector.DEPLOY_MOTOR_CAN_ID, true);

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    dCollectorM = deployMotor.getClosedLoopController();
    iCollectorM = intakeMotor.getClosedLoopController();
  }

  public Command collect(DoubleSupplier setPoint) {
    return new RunCommand(() -> {
      setIntakeSetPoint(setPoint.getAsDouble());
      setDeploySetPoint(setPoint.getAsDouble());
    }, this);
  }

  public void setIntakeSetPoint(double setPoint) {
    iCollectorM.setSetpoint(setPoint, ControlType.kPosition);
  }

  public void setDeploySetPoint(double setPoint) {
    dCollectorM.setSetpoint(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Collector/position", deployMotor.getEncoder().getPosition());
    Logger.recordOutput("Collector/velocity", deployMotor.getEncoder().getVelocity());
    Logger.recordOutput("Collector/voltage", deployMotor.getBusVoltage());
    Logger.recordOutput("Collector/dutycycle", deployMotor.getAppliedOutput());

    Logger.recordOutput("Intake/position", intakeMotor.getEncoder().getPosition());
    Logger.recordOutput("Intake/velocity", intakeMotor.getEncoder().getVelocity());
    Logger.recordOutput("Intake/voltage", intakeMotor.getBusVoltage());
    Logger.recordOutput("Intake/dutycycle", intakeMotor.getAppliedOutput());
  }

  public Command idle() {
    return new RunCommand(() -> {
      deployMotor.setVoltage(0);
      intakeMotor.setVoltage(0);
    }, this);
  }
}
