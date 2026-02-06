package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private SparkFlex leftshooter;
    private SparkFlex rightshooter;

    private SparkFlexConfig leftshooterConfig;
    private SparkFlexConfig rightshooterConfig;

    private SparkClosedLoopController leftmController;
    private double lastRPM = 0.0;
    private boolean recovering = false;
    private double boostEndTime = 0.0;
    private double targetRPM = 0;
    private static final double BOOST_DURATION_SEC = 2.0;
    private static final double BOOST_OUTPUT = 1.0;

    public Shooter() {
        leftshooter = new SparkFlex(Constants.Shooter.SHOOTER_1_CAN_ID, MotorType.kBrushless);
        rightshooter = new SparkFlex(Constants.Shooter.SHOOTER_2_CAN_ID, MotorType.kBrushless);

        leftshooterConfig = new SparkFlexConfig();
        leftshooterConfig.idleMode(IdleMode.kCoast);
        leftshooterConfig.smartCurrentLimit(80);
        leftshooterConfig.inverted(false);
        rightshooterConfig = new SparkFlexConfig();
        rightshooterConfig.follow(Constants.Shooter.SHOOTER_1_CAN_ID, true);
        leftshooterConfig.closedLoop.pid(0, 0, 0.001).feedForward.kV(0.000157);

        leftshooter.configure(leftshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightshooter.configure(rightshooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftmController = leftshooter.getClosedLoopController();
    }

    @Override
    public void periodic() {
        double currentRPM = leftshooter.getEncoder().getVelocity();
        double rpmDelta = currentRPM - lastRPM;

        Logger.recordOutput("Shooter/Velocity", currentRPM);
        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/RPMDelta", rpmDelta);

        Logger.recordOutput("Shooter/AppliedOutput", leftshooter.getAppliedOutput());
        Logger.recordOutput("Shooter/Voltage", leftshooter.getBusVoltage() * leftshooter.getAppliedOutput());
        Logger.recordOutput("Shooter/Current", leftshooter.getOutputCurrent());

        Logger.recordOutput("Shooter/Recovering", recovering);
        Logger.recordOutput("Shooter/BoostEndTime", boostEndTime);

        // Your existing logic...
        if (rpmDelta < -115.0 && !recovering) {
            recovering = true;
            boostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        }

        if (recovering && Timer.getFPGATimestamp() > boostEndTime) {
            recovering = false;
        }

        if (recovering) {
            leftshooter.set(BOOST_OUTPUT);
        } else {
            //leftshooter.set(BOOST_OUTPUT);
             leftmController.setSetpoint(targetRPM, ControlType.kVelocity);
        }

        lastRPM = currentRPM;
    }

    public Command idle() {
        return new RunCommand(() -> {
            targetRPM = 0;
        }, this);
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            targetRPM = setPoint.getAsDouble();
            // leftmController.setSetpoint(targetRPM, ControlType.kVelocity);
        }, this);
    }
}
