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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.EEUtil;

public class Deploy extends SubsystemBase {
  private SparkMax deployMotor;
  private AbsoluteEncoder absoluteEncoder;
  private SparkMaxConfig deployMotorConfig;
  public double deployMotorPower;
  private PIDController pidController;
  private double agitateHighPoint;
  public double collectorTarget;

  public Deploy() {
    deployMotor = new SparkMax(Constants.Deploy.DEPLOY_MOTOR_CAN_ID, MotorType.kBrushless);

    deployMotorConfig = new SparkMaxConfig();
    deployMotorConfig.idleMode(IdleMode.kBrake);
    deployMotorConfig.smartCurrentLimit(35);
    deployMotorConfig.inverted(false);
    deployMotorConfig.absoluteEncoder.zeroOffset(0.34); //make it just like before mechanical change

    deployMotor.configure(deployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pidController = new PIDController(Constants.Deploy.P, Constants.Deploy.I, Constants.Deploy.D);

    absoluteEncoder = deployMotor.getAbsoluteEncoder();
    collectorTarget = Constants.Deploy.COLLECT_SET_POINT; //start out
    agitateHighPoint = Constants.Deploy.AGITATE_HIGH;
    Logger.recordOutput(getName() + "/currentCommand", "notStarted");    
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

    deployMotorPower = EEUtil.clamp(-0.8, 0.8, 
    -Constants.Deploy.G * Math.sin(radiansFromRotation(absoluteEncoder.getPosition())) + 
      pidController.calculate(radiansFromRotation(absoluteEncoder.getPosition()), radiansFromRotation(collectorTarget)));

    if (Math.abs(collectorTarget - Constants.Deploy.COLLECT_SET_POINT) < 0.05 && Math.abs(absoluteEncoder.getPosition() - Constants.Deploy.COLLECT_SET_POINT) < 0.05) {
      deployMotorPower = -0.075;
    }
    //NEGATIVE IS GOING DOWN, POSITIVE IS UP

    deployMotor.set(deployMotorPower);
  }

   public Command idle() {
    return new InstantCommand(() -> {
      collectorTarget = Constants.Deploy.COLLECT_SET_POINT; //tuckedAway ? TUCKED_IN_SET_POINT : COLLECT_SET_POINT;
      Logger.recordOutput(getName() + "/currentCommand", "idle");
    }, this);
  }

  public Command setDeploy(DoubleSupplier target) {
    return new RunCommand(() -> {
      collectorTarget = target.getAsDouble();
      Logger.recordOutput(getName() + "/currentCommand", "collecting");
    }, this);
  }

  public Command collectorAway() {
    return new RunCommand(() -> {
      collectorTarget = Constants.Deploy.TUCKED_IN_SET_POINT;
      //tuckedAway = true;
      Logger.recordOutput(getName() + "/currentCommand", "collectorAway");
    }, this);
  }

  public Command collectorAdjust(DoubleSupplier reading) {
    return new RunCommand(() -> {
      collectorCalculate(reading.getAsDouble());
      Logger.recordOutput(getName() + "/currentCommand", "collectorAdjust");
    }, this);
  }
  
  private void collectorCalculate(double change) {
    collectorTarget = (Constants.Deploy.COLLECT_SET_POINT - Constants.Deploy.HALF_WAY_SET_POINT) * change + Constants.Deploy.HALF_WAY_SET_POINT;
  }

  /**
   * returns radians from the rotation of the collector, where 0 is upright. i made this because i am too tired to measure both
   */
  public double radiansFromRotation(double revolutions) {
    return Math.toRadians((revolutions - 0.692) * 360);
  }

  public void setCollector(double target) {
    collectorTarget = target;
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return absoluteEncoder;
  }
}
