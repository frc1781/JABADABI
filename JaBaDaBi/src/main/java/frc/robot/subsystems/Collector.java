package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.EEUtil;

public class Collector extends SubsystemBase {

  private SparkMax deployMotor;
  private TalonFX intakeMotor;
  private AbsoluteEncoder absoluteEncoder;

  private SparkMaxConfig deployMotorConfig;
  private TalonFXConfiguration intakeMotorConfig;
  private double intakeMotorPower;
  private double deployMotorPower;
  private PIDController pidController;

  private double collectorTarget;
  private boolean tuckedAway;

  public Collector() {

    deployMotor = new SparkMax(Constants.Collector.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    intakeMotor = new TalonFX(Constants.Collector.INTAKE_MOTOR_CAN_ID);

    intakeMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));

    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    intakeMotorPower = 0;
    deployMotorPower = 0;
    tuckedAway = true;

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(80);
    deployMotorConfig.inverted(false);
    deployMotorConfig.absoluteEncoder.zeroOffset(.9);

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pidController = new PIDController(Constants.Collector.P, Constants.Collector.I, Constants.Collector.D);

    absoluteEncoder = deployMotor.getAbsoluteEncoder();
    collectorTarget = 0.86; //start tucked away
  }

  private void collectorCalculate(double change) {
    collectorTarget = (0.321 - 0.650) * change + 0.650;
    //.865 at tucked .321 at deployed .650 at half way
  }

  public Command collect(DoubleSupplier setPoint) {
    return new RunCommand(() -> {
      intakeMotorPower = setPoint.getAsDouble();
    }, this);
  }

  public Command collectorUp(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorTarget += change.getAsDouble();
    }, this);
  }

  public Command collectorAdjust(DoubleSupplier reading) {
    return new RunCommand(() -> {
      tuckedAway = false;
      collectorCalculate(reading.getAsDouble());
    }, this);
  }

  public Command collectorDown(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorTarget -= change.getAsDouble();
    }, this);
  }

  public Command setCollector(DoubleSupplier setPoint) {
    return new RunCommand(() -> {
      collectorTarget = setPoint.getAsDouble();
    }, this);
  }

  public Command collectorAway() {
    return new RunCommand(() -> {
      tuckedAway = true;
    }, this);
  }

  public Command agitateFuel() {
    return new FunctionalCommand(
      () -> {
        collectorTarget = 0.4;
      },
      () -> {
        if (absoluteEncoder.getPosition() > 0.58 && absoluteEncoder.getPosition() < 0.62)
          collectorTarget = 0.4;
        if (absoluteEncoder.getPosition() > 0.40 && absoluteEncoder.getPosition() < 0.44)
          collectorTarget = 0.6;
      },
      (interrupted) -> {
        collectorTarget = 0.4;
      },
      () -> false,
      this);    
  }

  /**
   * returns radians from the rotation of the collector, where 0 is upright. i made this because i am too tired to measure both
   */
  public double radiansFromRotation(double revolutions) {
    return Math.toRadians((revolutions - 0.692) * 360);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Deploy/position", absoluteEncoder.getPosition());
    Logger.recordOutput("Deploy/positionInRadians", radiansFromRotation(absoluteEncoder.getPosition()));
    Logger.recordOutput("Deploy/velocity", deployMotor.getEncoder().getVelocity());
    Logger.recordOutput("Deploy/voltage", deployMotor.getBusVoltage() * deployMotor.getAppliedOutput());
    Logger.recordOutput("Deploy/targetposition", collectorTarget);
    Logger.recordOutput("Deploy/targetpositionInRadians", radiansFromRotation(collectorTarget));
    Logger.recordOutput("Deploy/dutycycle", deployMotor.getAppliedOutput());
    Logger.recordOutput("Deploy/power", deployMotorPower);

    Logger.recordOutput("Intake/position", intakeMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Intake/velocity", intakeMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Intake/dutycycle", intakeMotor.getDutyCycle().getValueAsDouble());

    deployMotorPower = EEUtil.clamp(-0.8, 0.8, -Constants.Collector.G * Math.sin(radiansFromRotation(deployMotor.getAbsoluteEncoder().getPosition())) + pidController.calculate(deployMotor.getAbsoluteEncoder().getPosition(), collectorTarget));

    deployMotor.set(deployMotorPower);
    intakeMotor.set(intakeMotorPower);
  }

  public Command idle() {
    return new RunCommand(() -> {
      collectorTarget = tuckedAway? 0.86 : 0.65;
    }, this);
  }
}
