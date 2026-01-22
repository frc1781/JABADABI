package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private SparkFlex leftshooter;
    private SparkFlex rightshooter;

    private SparkFlexConfig leftshooterConfig;
    private SparkFlexConfig rightshooterConfig;

    private SparkClosedLoopController leftmController;

    public Shooter() {
        leftshooter = new SparkFlex(Constants.SHOOTER_CAN_ID, MotorType.kBrushless);
        rightshooter = new SparkFlex(41, MotorType.kBrushless);

        leftshooterConfig = new SparkFlexConfig();
        leftshooterConfig.idleMode(IdleMode.kCoast);
        leftshooterConfig.smartCurrentLimit(40);
        leftshooterConfig.inverted(false);
        rightshooterConfig = new SparkFlexConfig();
        rightshooterConfig.follow(40, true);

        leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftmController = leftshooter.getClosedLoopController();

    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setMotorSetPoint(setPoint.getAsDouble());
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {

        leftmController.setSetpoint(setPoint, ControlType.kVelocity);

    }

}
