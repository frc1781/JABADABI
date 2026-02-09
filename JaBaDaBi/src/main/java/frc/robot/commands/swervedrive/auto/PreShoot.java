package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PreShoot extends Command {

    Shooter shooter;

    public PreShoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        shooter.shoot(() -> 5000);

    }

}
