package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import CRA.PIDTuning;

public class Shooter extends SubsystemBase {

    private TalonFX leftShooter;
    private TalonFX rightShooter;
    private SparkFlex motorHood;

    private TalonFXConfiguration leftShooterConfig;
    private TalonFXConfiguration rightShooterConfig;
    private SparkFlexConfig motorHoodConfig;

    private PIDTuning leftShooterPIDtuning;
    private PIDTuning rightShooterPIDtuning;
    private Slot0Configs leftShooterProfile;
    private Slot0Configs rightShooterProfile;
    private SparkClosedLoopController motorHoodController;

    private double leftLastRPM = 0.0;
    private boolean leftRecovering = false;
    private double leftBoostEndTime = 0.0;
    private double leftTargetRPM = 0;

    private double rightLastRPM = 0.0;
    private boolean rightRecovering = false;
    private double rightBoostEndTime = 0.0;
    private double rightTargetRPM = 0;

    private double fuelTimeOfFlight = 0.0;

    private static final double BOOST_DURATION_SEC = 0.1;
    private static final double BOOST_OUTPUT = 0.7;

    private final VelocityVoltage leftVelocityReq = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightVelocityReq = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut boostReq = new DutyCycleOut(BOOST_OUTPUT);

    private int desiredCurrent;

    public Shooter() {

        leftShooter = new TalonFX(Constants.Shooter.SHOOTER_1_CAN_ID);
        rightShooter = new TalonFX(Constants.Shooter.SHOOTER_2_CAN_ID);

        motorHood = new SparkFlex(Constants.Shooter.MOTORHOOD_CAN_ID, MotorType.kBrushless);

        leftShooterPIDtuning = new PIDTuning("left_shooter", 0, 0, 0, 0.0);
        rightShooterPIDtuning = new PIDTuning("right_shooter", 0, 0, 0, 0.0);

        leftShooterProfile = new Slot0Configs() // IDK YET EITHER
                .withKS(Constants.Shooter.S)
                .withKV(Constants.Shooter.V)
                .withKA(Constants.Shooter.A)
                .withKP(Constants.Shooter.P);

        rightShooterProfile = new Slot0Configs() // IDK YET EITHER
                .withKS(Constants.Shooter.S)
                .withKV(Constants.Shooter.V)
                .withKA(Constants.Shooter.A)
                .withKP(Constants.Shooter.P);

        leftShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSlot0(leftShooterProfile);

        rightShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withSlot0(rightShooterProfile);

        leftShooter.getConfigurator().apply(leftShooterConfig);
        rightShooter.getConfigurator().apply(rightShooterConfig);

        // motorHood.configure(motorHoodConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // motorHoodController = motorHood.getClosedLoopController();
    }

    @Override
    public void periodic() {
        leftShooter.set(leftTargetRPM / 7000.0);
        rightShooter.set(rightTargetRPM / 7000.0); // will remove later asldkhsadgkjssdfg

        // double leftCurrentRPM = leftShooter.getVelocity().getValueAsDouble();
        // double leftRpmDelta = leftCurrentRPM - leftLastRPM;

        // double rightCurrentRPM = rightShooter.getVelocity().getValueAsDouble();
        // double rightRpmDelta = rightCurrentRPM - rightLastRPM;

        // double leftCurrentRPM = leftShooter.getVelocity().getValueAsDouble() * 60.0;
        // double rightCurrentRPM = rightShooter.getVelocity().getValueAsDouble() * 60.0;
        // double leftRpmDelta = leftCurrentRPM - leftLastRPM;
        // double rightRpmDelta = rightCurrentRPM - rightLastRPM;
        // double now = Timer.getFPGATimestamp();

        // Logger.recordOutput("Shooter/leftVelocity", leftCurrentRPM);
        // Logger.recordOutput("Shooter/leftTargetRPM", leftTargetRPM);
        // Logger.recordOutput("Shooter/leftRPMDelta", leftRpmDelta);

        // Logger.recordOutput("Shooter/rightVelocity", rightCurrentRPM);
        // Logger.recordOutput("Shooter/rightTargetRPM", rightTargetRPM);
        // Logger.recordOutput("Shooter/rightRPMDelta", rightRpmDelta);

        Logger.recordOutput("Shooter/leftVoltage", leftShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/leftCurrent", leftShooter.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Shooter/rightVoltage", rightShooter.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Shooter/rightCurrent", rightShooter.getSupplyCurrent().getValueAsDouble());

        Logger.recordOutput("Shooter/leftRecovering", leftRecovering);
        Logger.recordOutput("Shooter/leftBoostEndTime", leftBoostEndTime);

        Logger.recordOutput("Shooter/rightRecovering", rightRecovering);
        Logger.recordOutput("Shooter/rightBoostEndTime", rightBoostEndTime);

        // if (leftRpmDelta < -50.0 && !leftRecovering) {
        // leftRecovering = true;
        // leftBoostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        // } else if (leftRecovering && Timer.getFPGATimestamp() > leftBoostEndTime) {
        // leftRecovering = false;
        // }

        // if (leftRecovering) {
        // leftShooter.set(BOOST_OUTPUT);
        // } else {
        // leftShooter.setControl(new VelocityVoltage(leftTargetRPM/60).withSlot(0));
        // }
        // if (!leftRecovering && leftTargetRPM == 0) {
        // leftShooter.set(0);
        // }

        // if (rightRpmDelta < -50.0 && !rightRecovering) {
        // rightRecovering = true;
        // rightBoostEndTime = Timer.getFPGATimestamp() + BOOST_DURATION_SEC;
        // } else if (leftRecovering && Timer.getFPGATimestamp() > rightBoostEndTime) {
        // rightRecovering = false;
        // }

        // if (rightRecovering) {
        // rightShooter.set(BOOST_OUTPUT);
        // } else {
        // rightShooter.setControl(new VelocityVoltage(rightTargetRPM/60).withSlot(0));
        // }
        // if (!leftRecovering && rightTargetRPM == 0) {
        // rightShooter.set(0);
        // }

        // leftLastRPM = leftCurrentRPM;
        // rightLastRPM = rightCurrentRPM;

        // LEFT BOOST LOGIC
        // if (leftRpmDelta < -50.0 && !leftRecovering) {
        //     leftRecovering = true;
        //     leftBoostEndTime = now + BOOST_DURATION_SEC;
        // } else if (leftRecovering && now > leftBoostEndTime) {
        //     leftRecovering = false;
        // }
        // if (leftRecovering) {
        //     leftShooter.setControl(boostReq);
        // } else if (leftTargetRPM == 0) {
        //     leftShooter.set(0);
        // } else {
        //     leftShooter.setControl(leftVelocityReq.withVelocity(leftTargetRPM / 60.0));
        // }
        // // RIGHT BOOST LOGIC
        // if (rightRpmDelta < -50.0 && !rightRecovering) {
        //     rightRecovering = true;
        //     rightBoostEndTime = now + BOOST_DURATION_SEC;
        // } else if (rightRecovering && now > rightBoostEndTime)

        // {
        //     rightRecovering = false;
        // }
        // if (rightRecovering) {
        //     rightShooter.setControl(boostReq);
        // } else if (rightTargetRPM == 0) {
        //     rightShooter.set(0);
        // } else {
        //     rightShooter.setControl(rightVelocityReq.withVelocity(rightTargetRPM /
        //             60.0));
        // }
        // leftLastRPM = leftCurrentRPM;
        // rightLastRPM = rightCurrentRPM;
    }

    public Command idle() {
        return new RunCommand(() -> {
            leftTargetRPM = 0;
            rightTargetRPM = 0;
            leftRecovering = false;
            rightRecovering = false;
        }, this);
    }

    public Command shoot(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            leftTargetRPM = setPoint.getAsDouble();
            rightTargetRPM = setPoint.getAsDouble();
        }, this);
    }

    private void reconfigure(int current) {
        if (desiredCurrent == current) {
            return;
        }
        desiredCurrent = current;
        leftShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(desiredCurrent))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive));

        rightShooterConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(desiredCurrent))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive));

        leftShooter.getConfigurator().apply(leftShooterConfig);
        rightShooter.getConfigurator().apply(rightShooterConfig);

        leftShooterPIDtuning = new PIDTuning("left_shooter", 0, 0, 0, 0.0);
        rightShooterPIDtuning = new PIDTuning("right_shooter", 0, 0, 0, 0.0);

        leftShooterProfile = new Slot0Configs() // IDK YET EITHER
                .withKS(Constants.Shooter.S)
                .withKV(Constants.Shooter.V)
                .withKA(Constants.Shooter.A)
                .withKP(Constants.Shooter.P);

        rightShooterProfile = new Slot0Configs() // IDK YET EITHER
                .withKS(Constants.Shooter.S)
                .withKV(Constants.Shooter.V)
                .withKA(Constants.Shooter.A)
                .withKP(Constants.Shooter.P);

        leftShooter.getConfigurator().apply(leftShooterProfile);
        rightShooter.getConfigurator().apply(rightShooterProfile);

    }

    public Command motorReconfig() {
        return new InstantCommand(() -> {
            reconfigure(80); // we will move this to 120 later
            System.out.println("left_shooter P: " + leftShooterPIDtuning.getPID()[0]);
            System.out.println("left_shooter I: " + leftShooterPIDtuning.getPID()[1]);
            System.out.println("left_shooter D: " + leftShooterPIDtuning.getPID()[2]);
            System.out.println("right_shooter P: " + rightShooterPIDtuning.getPID()[0]);
            System.out.println("right_shooter I: " + rightShooterPIDtuning.getPID()[1]);
            System.out.println("right_shooter D: " + rightShooterPIDtuning.getPID()[2]);
        });
    }

    public Command adjustHood(DoubleSupplier setPoint) {
        return new RunCommand(() -> {
            // motorHoodController.setSetpoint(leftTargetRPM, ControlType.kPosition);
            // motorHoodController.setSetpoint(rightTargetRPM, ControlType.kPosition);
        }, this);
    }

    public double getFuelTimeOfFlight() {
        return fuelTimeOfFlight;
    }
}
