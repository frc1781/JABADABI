package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Intake;

public class Reject extends SequentialCommandGroup {
    public Reject(Deploy deploy, Intake intake, Loader loader, Conveyor conveyor, Shooter shooter) {
        addRequirements(deploy, loader, conveyor, shooter);
        addCommands(
                deploy.setDeploy(() -> Constants.Deploy.HALF_WAY_SET_POINT),
                new ParallelCommandGroup(
                        intake.runIntake(() -> -1), 
                        deploy.setDeploy(() -> 0.135),
                        loader.runLoader(() -> -0.7),
                        conveyor.unloadFuel(() -> true)),
                        shooter.shoot(() -> -50)
                );
    }
}
