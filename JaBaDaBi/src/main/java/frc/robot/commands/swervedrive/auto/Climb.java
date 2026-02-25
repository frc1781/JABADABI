package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lights;

public class Climb extends Command {
    Timer timer;
    Lights lights;
    Climber climber;

    public Climb(Climber climber, Lights lights) {
        this.climber = climber;
        this.lights = lights;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        System.out.println(getName() + "started");
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        climber.ascend();
        lights.run(Lights.Colors.BLUE, Lights.Patterns.FAST_FLASH);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 10 || 
        (climber.getLeftEncoder().getPosition() > 20 && climber.getRightEncoder().getPosition() > 20);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println(getName() + "interrupted");
        } else {
            System.out.println(getName() + "ended");
        }
        lights.set(Lights.Special.RAINBOW);
        timer.stop();
        timer.reset();
    }
}
