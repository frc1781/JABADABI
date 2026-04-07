package frc.robot.commands.swervedrive.autocommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class StopAll extends SequentialCommandGroup {

    public StopAll(Loader loader, Conveyor conveyor, Shooter shooter, Collector collector, DoubleSupplier speed) {
        addCommands(
            new ParallelCommandGroup(
                shooter.shoot(speed),
                loader.runLoader(() -> -1),
                conveyor.convey(() -> -1),
                collector.idle()
            )
        );
    }

}
