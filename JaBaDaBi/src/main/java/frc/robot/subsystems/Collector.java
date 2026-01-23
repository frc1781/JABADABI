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

  private SparkMax dCollectorMotor;
  private SparkMax iCollectorMotor;

  private SparkMaxConfig dCollectMotorConfig;
  private SparkMaxConfig iCollectMotorConfig;
  private SparkClosedLoopController iCollectorM;
  private SparkClosedLoopController dCollectorM;

  public Collector() {

    dCollectorMotor = new SparkMax(Constants.Collector.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    iCollectorMotor = new SparkMax(Constants.Collector.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    dCollectMotorConfig = new SparkMaxConfig();
    dCollectMotorConfig.idleMode(IdleMode.kBrake);
    dCollectMotorConfig.smartCurrentLimit(40);
    dCollectMotorConfig.inverted(false);
    iCollectMotorConfig = new SparkMaxConfig();
    iCollectMotorConfig.follow(34, true);

    dCollectorMotor.configure(dCollectMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    iCollectorMotor.configure(iCollectMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    iCollectorM = iCollectorMotor.getClosedLoopController();
    dCollectorM = iCollectorMotor.getClosedLoopController();
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
    Logger.recordOutput("Collector/position", dCollectorMotor.getEncoder().getPosition());
    Logger.recordOutput("Collector/velocity", dCollectorMotor.getEncoder().getVelocity());
    Logger.recordOutput("Collector/voltage", dCollectorMotor.getBusVoltage());
    Logger.recordOutput("Collector/dutycycle", dCollectorMotor.getAppliedOutput());

    Logger.recordOutput("Intake/position", iCollectorMotor.getEncoder().getPosition());
    Logger.recordOutput("Intake/velocity", iCollectorMotor.getEncoder().getVelocity());
    Logger.recordOutput("Intake/voltage", iCollectorMotor.getBusVoltage());
    Logger.recordOutput("Intake/dutycycle", iCollectorMotor.getAppliedOutput());
  }
}
