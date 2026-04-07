package frc.robot.commands.swervedrive.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Deploy;

public class Unjam extends SequentialCommandGroup {
    public Unjam(Deploy deploy, Loader loader, Conveyor conveyor, Shooter shooter) {
        addRequirements(deploy, loader, conveyor, shooter);
        addCommands(
                deploy.deploy(() -> deploy.HALF_WAY_SET_POINT),
                new ParallelCommandGroup(
                        deploy.deploy(() -> 0.155),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true))
                        //shooter.shoot(() -> -50)
                );
    }
}
