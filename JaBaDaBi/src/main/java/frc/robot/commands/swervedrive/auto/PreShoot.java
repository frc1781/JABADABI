package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;

public class PreShoot extends SequentialCommandGroup {

    Shooter shooter;

    public PreShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        addCommands(
            new ParallelRaceGroup(
            shooter.shoot(() -> 55)
            )
        );
    }

}
