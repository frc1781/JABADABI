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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import CRA.FeedForwardTuning;
import CRA.PIDTuning;

public class Shooter extends SubsystemBase {

    private SparkFlex leftShooter;
    private SparkFlex rightShooter;

    private SparkFlexConfig leftShooterConfig;
    private SparkFlexConfig rightShooterConfig;

    private PIDController shooterPID;
    private PIDTuning shooterPIDtuning;
    private SparkClosedLoopController leftController;
    private double lastRPM = 0.0;
    private boolean recovering = false;
    private double boostEndTime = 0.0;
    private double targetRPM = 0;
    private static final double BOOST_DURATION_SEC = 0.1;
    private static final double BOOST_OUTPUT = 0.7;

    public Shooter() {
        leftShooter = new SparkFlex(Constants.Shooter.SHOOTER_1_CAN_ID, MotorType.kBrushless);
        rightShooter = new SparkFlex(Constants.Shooter.SHOOTER_2_CAN_ID, MotorType.kBrushless);

        shooterPIDtuning = new PIDTuning("shooter", 0, 0, 0, 0.0);

        leftShooterConfig = new SparkFlexConfig();
        leftShooterConfig.idleMode(IdleMode.kCoast);
        leftShooterConfig.smartCurrentLimit(80);
        leftShooterConfig.inverted(false);
        rightShooterConfig = new SparkFlexConfig();
        rightShooterConfig.follow(Constants.Shooter.SHOOTER_1_CAN_ID, true);
        leftShooterConfig.closedLoop.pid(
                shooterPIDtuning.getPID()[0],
                shooterPIDtuning.getPID()[1],
                shooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

        leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooter.configure(rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftController = leftShooter.getClosedLoopController();
    }

    @Override
    public void periodic() {
        double currentRPM = leftShooter.getEncoder().getVelocity();
        double rpmDelta = currentRPM - lastRPM;

        Logger.recordOutput("Shooter/Velocity", currentRPM);
        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/RPMDelta", rpmDelta);

        Logger.recordOutput("Shooter/AppliedOutput", leftShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/Voltage", leftShooter.getBusVoltage() * leftShooter.getAppliedOutput());
        Logger.recordOutput("Shooter/Current", leftShooter.getOutputCurrent());

        Logger.recordOutput("Shooter/Recovering", recovering);
        Logger.recordOutput("Shooter/BoostEndTime", boostEndTime);

        if (rpmDelta < -50.0 && !recovering && targetRPM > 60000000 && currentRPM > 2500) {
            recovering = true;
            boostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        }

        if (recovering && Timer.getFPGATimestamp() > boostEndTime) {
            recovering = false;
        }

        if (recovering) {
            leftShooter.set(BOOST_OUTPUT);
        } else {
            // leftShooter.set(BOOST_OUTPUT);
            // leftShooter.set(targetRPM * 0.000157);
            leftController.setSetpoint(targetRPM, ControlType.kVelocity);
        }
        if (!recovering && targetRPM == 0) {
            leftShooter.set(0);
            // leftController.setSetpoint(0, ControlType.kDutyCycle);
            lastRPM = currentRPM;
            return;
        }

        lastRPM = currentRPM;
    }

    public Command idle() {
        return new RunCommand(() -> {
            targetRPM = 0;
            recovering = false;
            // leftController.setSetpoint(0, ControlType.kDutyCycle);
        }, this);
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            targetRPM = setPoint.getAsDouble();
            // leftController.setSetpoint(targetRPM, ControlType.kVelocity);
        }, this);
    }

    public Command motorReconfig() {
        return new InstantCommand(() -> {
            leftShooterConfig.idleMode(IdleMode.kCoast);
            leftShooterConfig.smartCurrentLimit(80);
            leftShooterConfig.inverted(false);

            leftShooterConfig.closedLoop.pid(
                    shooterPIDtuning.getPID()[0],
                    shooterPIDtuning.getPID()[1],
                    shooterPIDtuning.getPID()[2]).feedForward.kV(0.000157);

            leftShooter.configure(leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            System.out.println("shooter P: " +  shooterPIDtuning.getPID()[0]);
            System.out.println("shooter I: " + shooterPIDtuning.getPID()[1]);
            System.out.println("shooter D: " + shooterPIDtuning.getPID()[2]);
        });
    }
}
