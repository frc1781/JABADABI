package frc.robot.commands.telecommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class Shooting {

    public static Command shotTesting() {
        return new RunCommand(() -> {
            Shooter.leftReqRPS = 55;
            Shooter.rightReqRPS = 55;
        });
    }
}
