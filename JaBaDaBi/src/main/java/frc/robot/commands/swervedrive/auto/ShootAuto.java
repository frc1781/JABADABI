package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;

public class ShootAuto extends SequentialCommandGroup {

    public ShootAuto(Loader loader, Conveyor conveyor, Shooter shooter, double duration) {
        addCommands(
                shooter.shooterToSpeed(() -> 55),
                new ParallelDeadlineGroup(
                        new WaitCommand(duration),
                        shooter.shoot(() -> 55),
                        loader.runLoader(() -> 1),
                        conveyor.loadFuel(() -> true)),
                shooter.idle(),
                loader.idle(),
                conveyor.idle());
    }

}
