package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Intake;

public class Unjam extends SequentialCommandGroup {
    public Unjam(Deploy deploy, Intake intake, Loader loader, Conveyor conveyor) {
        addRequirements(deploy, intake, loader, conveyor);
        addCommands(
                deploy.setDeploy(() -> Constants.Deploy.HALF_WAY_SET_POINT),
                new ParallelCommandGroup(
                        deploy.setDeploy(() -> 0.155),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true))
                );
    }
}
