package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private TalonFX intakeMotor;

  private SparkMaxConfig deployMotorConfig;
  private TalonFXConfiguration intakeMotorConfig;
  private SparkClosedLoopController iCollectorM;
  private SparkClosedLoopController dCollectorM;
  private double intakeMotorPower;
  private double deployMotorPower;

  public Collector() {

    deployMotor = new SparkMax(Constants.Collector.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
     intakeMotor = new TalonFX(Constants.Collector.INTAKE_MOTOR_CAN_ID);

        intakeMotorConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
                //invert motor
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast));


        intakeMotor.getConfigurator().apply(intakeMotorConfig);

    intakeMotorPower = 0;
    deployMotorPower = 0;

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(40);
    deployMotorConfig.inverted(false);

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    dCollectorM = deployMotor.getClosedLoopController();

  }

  public Command collect(DoubleSupplier setPoint) {
    return new RunCommand(() -> {
      //setIntakeSetPoint(setPoint.getAsDouble());
      intakeMotorPower = setPoint.getAsDouble();
    //setDeploySetPoint(setPoint.getAsDouble());
    }, this);
  }

  // public Command deployCollect(DoubleSupplier setPoint) {
  // return new RunCommand(() -> {
  // setDeploySetPoint(setPoint.getAsDouble());
  // }, this);
  // }

  public void setIntakeSetPoint(double setPoint) {
    iCollectorM.setSetpoint(setPoint, ControlType.kPosition);
  }

  public void setDeploySetPoint(double setPoint) {
    dCollectorM.setSetpoint(setPoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Collector/position", deployMotor.getEncoder().getPosition());
    // Logger.recordOutput("Collector/velocity", deployMotor.getEncoder().getVelocity());
    // Logger.recordOutput("Collector/voltage", deployMotor.getBusVoltage());
    // Logger.recordOutput("Collector/dutycycle", deployMotor.getAppliedOutput());

    // Logger.recordOutput("Intake/position", intakeMotor.getEncoder().getPosition());
    // Logger.recordOutput("Intake/velocity", intakeMotor.getEncoder().getVelocity());
    // Logger.recordOutput("Intake/voltage", intakeMotor.getBusVoltage());
    // Logger.recordOutput("Intake/dutycycle", intakeMotor.getAppliedOutput());
    Logger.recordOutput("Intake/intakeMotor", intakeMotorPower);
    //deployMotor.set(deployMotorPower);
    System.out.println("Intake Collector " + intakeMotorPower);
    intakeMotor.set(intakeMotorPower);
  }

  public Command idle() {
    return new RunCommand(() -> {
     // deployMotor.setVoltage(0);
      //intakeMotor.setVoltage(0);
      intakeMotorPower = 0;
    }, this);
  }
}
