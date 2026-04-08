package frc.robot.commands.swervedrive.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class StopAll extends ParallelCommandGroup {

    public StopAll(Loader loader, Conveyor conveyor, Shooter shooter, Deploy collector, DoubleSupplier speed) {
        addCommands(
                // shooter.shoot(speed),
                loader.runLoader(() -> -1),
                conveyor.convey(() -> -1),
                collector.idle());
    }

}
