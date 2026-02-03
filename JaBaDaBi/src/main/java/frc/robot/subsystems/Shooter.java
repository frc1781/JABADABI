package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
        leftshooter = new SparkFlex(Constants.Shooter.SHOOTER_1_CAN_ID, MotorType.kBrushless);
        rightshooter = new SparkFlex(Constants.Shooter.SHOOTER_2_CAN_ID, MotorType.kBrushless);

        leftshooterConfig = new SparkFlexConfig();
        leftshooterConfig.idleMode(IdleMode.kCoast);
        leftshooterConfig.smartCurrentLimit(40);
        leftshooterConfig.inverted(false);
        rightshooterConfig = new SparkFlexConfig();
        rightshooterConfig.follow(Constants.Shooter.SHOOTER_1_CAN_ID, true);

        leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftmController = leftshooter.getClosedLoopController();

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/position", leftshooter.getEncoder().getPosition());
        Logger.recordOutput("Shooter/velocity", leftshooter.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/voltage", leftshooter.getBusVoltage());
        Logger.recordOutput("Shooter/dutycycle", leftshooter.getAppliedOutput());
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            setMotorSetPoint(1000);//setPoint.getAsDouble()); 
            System.out.println("Aaron is cool Antonio is not");
        }, this);
    }

    public void setMotorSetPoint(double setPoint) {
        //leftmController.setSetpoint(setPoint, ControlType.kVelocity);
        leftshooter.set(.5);
    }

}
