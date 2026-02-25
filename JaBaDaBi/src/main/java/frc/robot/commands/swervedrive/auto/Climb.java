package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.NeonLights;

public class Climb extends Command {
    Timer timer;
    NeonLights lights;

    public Climb(NeonLights lights) {
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
        //lights.run(Lights.Colors.BLUE, Lights.Patterns.FAST_FLASH);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 5;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println(getName() + "interrupted");
        } else {
            System.out.println(getName() + "ended");
        }
        timer.stop();
        timer.reset();
    }
}
