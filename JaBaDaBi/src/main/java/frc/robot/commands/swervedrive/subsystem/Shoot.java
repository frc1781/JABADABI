package frc.robot.commands.swervedrive.subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class Shoot extends ParallelCommandGroup{

    public Shoot(Loader loader, Conveyor conveyor, Shooter shooter, Collector collector, DoubleSupplier speed) {
        //addRequirements(collector, loader, conveyor, shooter);
        addCommands(
                shooter.shoot(speed),
                loader.runLoader(() -> shooter.atSpeed() ? 0.95 : 0.0),
                conveyor.convey(() -> shooter.atSpeed() ? 1 : 0),
                collector.agitateFuel()
            );
    }
}
