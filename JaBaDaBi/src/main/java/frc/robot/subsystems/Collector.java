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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.EEUtil;

public class Collector extends SubsystemBase {

  private SparkMax deployMotor;
  private TalonFX intakeMotor;
  private final AbsoluteEncoder abs;
  private double offsetDeg;

  private SparkMaxConfig deployMotorConfig;
  private TalonFXConfiguration intakeMotorConfig;
  private double intakeMotorPower;
  private double deployMotorPower;
  private ArmFeedforward feedforwardController;
  private PIDController pidController;

  private double collectorTarget;

  public Collector() {

    deployMotor = new SparkMax(Constants.Collector.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);
    intakeMotor = new TalonFX(Constants.Collector.INTAKE_MOTOR_CAN_ID);
    abs = deployMotor.getAbsoluteEncoder();

    intakeMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));

    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    intakeMotorPower = 0;
    deployMotorPower = 0;
    offsetDeg = 0;

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(40);
    deployMotorConfig.inverted(false);
    deployMotorConfig.absoluteEncoder.zeroOffset(.9);

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pidController = new PIDController(Constants.Collector.P, Constants.Collector.I, Constants.Collector.D);
    collectorTarget = deployMotor.getAbsoluteEncoder().getPosition();

  }

  private void collectorCalculate(double change) {
    collectorTarget = -.29 * change + .65; // values need to be change to fit robot exact
    //.86 at tucked .36 at deployed .65 at half way
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

  public Command collectorAdjust(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorCalculate(change.getAsDouble());
    }, this);
  }

  public Command collectorDown(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorTarget -= change.getAsDouble();
    }, this);
  }

  public Command collectorAway() {
    return new RunCommand(() -> {
      collectorTarget = .86;
    }, this);
  }

  /**
   * returns radians from the rotation of the collector, where 0 is upright
   */
  public double radiansFromRotation(double revolutions) {
    return Math.toRadians((revolutions - 0.65) * 360);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Deploy/position", deployMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Deploy/velocity", deployMotor.getEncoder().getVelocity());
    Logger.recordOutput("Deploy/voltage", deployMotor.getBusVoltage() * deployMotor.getAppliedOutput());
    Logger.recordOutput("Deploy/targetposition", collectorTarget);
    Logger.recordOutput("Deploy/dutycycle", deployMotor.getAppliedOutput());
    Logger.recordOutput("Deploy/power", deployMotorPower);

    Logger.recordOutput("Intake/position", intakeMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Intake/velocity", intakeMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Intake/voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Intake/dutycycle", intakeMotor.getDutyCycle().getValueAsDouble());

    deployMotorPower = Constants.Collector.kG * Math.cos(radiansFromRotation(collectorTarget) - radiansFromRotation(deployMotor.getAbsoluteEncoder().getPosition())) + pidController.calculate(deployMotor.getAbsoluteEncoder().getPosition(), collectorTarget);

    deployMotor.set(deployMotorPower);
    intakeMotor.set(intakeMotorPower);
  }

  public Command idle(DoubleSupplier idle) {
    return new RunCommand(() -> {
      // collectorTarget = idle.getAsDouble(); 
    }, this);
  }
}
