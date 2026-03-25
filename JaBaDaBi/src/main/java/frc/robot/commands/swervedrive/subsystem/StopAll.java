package frc.robot.commands.swervedrive.subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class StopAll extends SequentialCommandGroup {

    public StopAll(Loader loader, Conveyor conveyor, Shooter shooter, Collector collector, DoubleSupplier speed) {
        addRequirements(collector, loader, conveyor, shooter);
        addCommands(
            new ParallelCommandGroup(
                shooter.shoot(speed),
                loader.runLoader(() -> 0),
                conveyor.convey(() -> 0),
                collector.idle()
            )
        );
    }

}
