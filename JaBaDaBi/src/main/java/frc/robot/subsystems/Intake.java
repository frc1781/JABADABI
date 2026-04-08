package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeMotorConfig;
  public double intakeMotorPower;

  public Intake() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_CAN_ID);

    intakeMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    intakeMotorPower = 0;
    Logger.recordOutput(getName() + "/currentCommand", "notStarted");    
  }

  @Override
  public void periodic() {
    Logger.recordOutput(getName() + "/Intake/position", intakeMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/velocity", intakeMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/dutycycle", intakeMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(getName() + "/Intake/motorPower", intakeMotorPower);

    intakeMotor.set(intakeMotorPower);
  }

  public Command idle() {
    return new InstantCommand(() -> {
      intakeMotorPower = 0;
      Logger.recordOutput(getName() + "/currentCommand", "idle");
    }, this);
  }

  public Command collect() {
    return new RunCommand(() -> {
      intakeMotorPower = 1.0;
      Logger.recordOutput(getName() + "/currentCommand", "collecting");
    }, this);
  }

  public Command runIntake(DoubleSupplier setPoint) {
    return new InstantCommand(() -> {
      intakeMotorPower = setPoint.getAsDouble();
      Logger.recordOutput(getName() + "/currentCommand", "runningIntake at: " + intakeMotorPower);
    });
  }

  public void setIntake(double setPoint) {
    intakeMotorPower = setPoint;
  }

  
}
