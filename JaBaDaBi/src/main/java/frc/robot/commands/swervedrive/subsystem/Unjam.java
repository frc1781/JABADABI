package frc.robot.commands.swervedrive.subsystem;

import java.time.Duration;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Collector;

public class Unjam extends SequentialCommandGroup {
    Collector collector;
    Loader loader;
    Conveyor conveyor;

    public Unjam(Collector collector, Loader loader, Conveyor conveyor) {
        addRequirements(collector, loader, conveyor);
        addCommands(
                collector.deploy(() -> collector.HALF_WAY_SET_POINT),
                new ParallelCommandGroup(
                        collector.intakeSetMotorPower(() -> 1),
                        collector.deploy(() -> 0.33),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true))
                );
    }
}
