package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {

    private TalonFX motor;

    private TalonFXConfiguration loaderConfig;

    private Slot0Configs loaderProfile;

    private double motorPower;

    public Loader() {
        motor = new TalonFX(Constants.Loader.MOTOR_CAN_ID);

        loaderProfile = new Slot0Configs() // IDK YET EITHER
                .withKS(Constants.Loader.S)
                .withKV(Constants.Loader.V)
                .withKA(Constants.Loader.A)
                .withKP(Constants.Loader.P);

        loaderConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
                //invert motor
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(loaderProfile); // IDK YET

        motor.getConfigurator().apply(loaderConfig);
    }

    public void periodic() {
        motor.set(motorPower);
        Logger.recordOutput("Loader/motorPower", motorPower);
        System.out.println();
    }

    public Command runLoader(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            motorPower = setPoint.getAsDouble();
            //setMotorSetPoint(setPoint.getAsDouble());
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {
        motor.setControl(new VelocityVoltage(setPoint).withSlot(0));
    }

    public Command idle() {
        return new RunCommand(() -> {
            motorPower = 0;
        }, this);
    }
}