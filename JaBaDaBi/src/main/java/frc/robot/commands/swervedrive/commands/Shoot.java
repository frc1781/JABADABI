package frc.robot.commands.swervedrive.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class Shoot extends ParallelCommandGroup{

    public Shoot(Loader loader, Intake intake, Conveyor conveyor, Shooter shooter, Deploy deploy, DoubleSupplier speed) {
        addRequirements(deploy, intake, loader, conveyor);
        addCommands(
                shooter.shoot(speed),
                loader.runLoader(() -> shooter.atSpeed() ? 0.95 : 0.0),
                conveyor.convey(() -> shooter.atSpeed() ? 1 : 0),
                new AgitateFuel(deploy, intake)
            );
    }
}
