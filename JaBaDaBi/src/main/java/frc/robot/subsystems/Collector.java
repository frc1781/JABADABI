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
  private final double COLLECT_SET_POINT = 0.36;
  private final double TUCKED_IN_SET_POINT = 0.86;
  private final double HALF_WAY_SET_POINT = 0.65;
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
    tuckedAway = false;

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(35);
    deployMotorConfig.inverted(false);
    deployMotorConfig.absoluteEncoder.zeroOffset(0.406); //make it just like before mechanical change

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pidController = new PIDController(Constants.Collector.P, Constants.Collector.I, Constants.Collector.D);

    absoluteEncoder = deployMotor.getAbsoluteEncoder();
    collectorTarget = COLLECT_SET_POINT; //start out
    Logger.recordOutput(getName() + "/currentCommand", "notStarted");    
  }

  private void collectorCalculate(double change) {
    collectorTarget = (COLLECT_SET_POINT - HALF_WAY_SET_POINT) * change + HALF_WAY_SET_POINT;
  }
  
  public Command collect() {
    return new RunCommand(() -> {
      intakeMotorPower = 1.0;
      collectorTarget = COLLECT_SET_POINT;
    }, this);
  }

  public Command intakeSetMotorPower(DoubleSupplier setPoint) {
    return new InstantCommand(() -> {
      intakeMotorPower = setPoint.getAsDouble();
    });
  }

  public Command collectorUp(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorTarget += change.getAsDouble();
      Logger.recordOutput(getName() + "/currentCommand", "collectorUp");
    }, this);
  }

  public Command collectorAdjust(DoubleSupplier reading) {
    return new RunCommand(() -> {
      tuckedAway = false;
      collectorCalculate(reading.getAsDouble());
      Logger.recordOutput(getName() + "/currentCommand", "collectorAdjust");
    }, this);
  }

  public Command collectorDown(DoubleSupplier change) {
    return new RunCommand(() -> {
      collectorTarget -= change.getAsDouble();
      Logger.recordOutput(getName() + "/currentCommand", "collectorDown");
    }, this);
  }

  public Command setCollector(DoubleSupplier setPoint) {
    return new InstantCommand(() -> {
      collectorTarget = setPoint.getAsDouble();
      Logger.recordOutput(getName() + "/currentCommand", "setCollector");
    }, this);
  }

  public Command collectorAway() {
    return new RunCommand(() -> {
      tuckedAway = true;
      Logger.recordOutput(getName() + "/currentCommand", "collectorAway");
    }, this);
  }

  public Command deploy(DoubleSupplier deploy){
    return new InstantCommand(() -> {
      tuckedAway = false;
      collectorTarget = deploy.getAsDouble();
      intakeMotorPower = 1.0;
      Logger.recordOutput(getName() + "/currentCommand", "deploy");
    }, this);
  }

  public Command agitateFuel() {
    return new FunctionalCommand(
      () -> {
        collectorTarget = 0.4;
        Logger.recordOutput(getName() + "/currentCommand", "agitateFuel");
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
    Logger.recordOutput(getName() + "/Deploy/relativeEncoder", deployMotor.getEncoder().getPosition());
    Logger.recordOutput(getName() + "/Deploy/absoluteEncoderPosition", absoluteEncoder.getPosition());
    Logger.recordOutput(getName() + "/Deploy/velocity", absoluteEncoder.getVelocity());
    Logger.recordOutput(getName() + "/Deploy/voltage", deployMotor.getBusVoltage() * deployMotor.getAppliedOutput());
    Logger.recordOutput(getName() + "/Deploy/collectorTarget", collectorTarget);
    Logger.recordOutput(getName() + "/Deploy/convertedCurrentPositionRadians", radiansFromRotation(absoluteEncoder.getPosition()));
    Logger.recordOutput(getName() + "/Deploy/convertedTargetPositionRadians", radiansFromRotation(collectorTarget));
    Logger.recordOutput(getName() + "/Deploy/dutycycle", deployMotor.getAppliedOutput());
    Logger.recordOutput(getName() + "/Deploy/current", deployMotor.getOutputCurrent());
    Logger.recordOutput(getName() + "/Deploy/tuckedAway", tuckedAway);
    Logger.recordOutput(getName() + "/Intake/position", intakeMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/velocity", intakeMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/dutycycle", intakeMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/motorPower", intakeMotorPower);

    deployMotorPower = EEUtil.clamp(-0.8, 0.8, 
    -Constants.Collector.G * Math.sin(radiansFromRotation(absoluteEncoder.getPosition())) + 
      pidController.calculate(radiansFromRotation(absoluteEncoder.getPosition()), radiansFromRotation(collectorTarget)));

    deployMotor.set(deployMotorPower);
    intakeMotor.set(intakeMotorPower);
  }

  public Command idle() {
    return new InstantCommand(() -> {
      collectorTarget = tuckedAway ? TUCKED_IN_SET_POINT : COLLECT_SET_POINT;
      intakeMotorPower = 0;
      Logger.recordOutput(getName() + "/currentCommand", "idle");
    }, this);
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return absoluteEncoder;
  }
}
