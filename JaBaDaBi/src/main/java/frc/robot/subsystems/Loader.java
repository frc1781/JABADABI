package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

    private TalonFX loaderMotor;

    private TalonFXConfiguration loaderConfig;

    private Slot0Configs loaderProfile;

    public Loader() {
        loaderMotor = new TalonFX(Constants.Loader.MOTOR_CAN_ID);

        loaderConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(40))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive));  //IDK YET

        loaderMotor.getConfigurator().apply(loaderConfig);

        loaderProfile = new Slot0Configs()  //IDK YET EITHER
            .withKS(Constants.Loader.S)
            .withKV(Constants.Loader.V)
            .withKA(Constants.Loader.A)
            .withKP(Constants.Loader.P);

        loaderMotor.getConfigurator().apply(loaderProfile);
    }

    public Command runLoader(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setMotorSetPoint(setPoint.getAsDouble());
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {
        loaderMotor.setControl(new VelocityVoltage(setPoint).withSlot(0));
    }

}