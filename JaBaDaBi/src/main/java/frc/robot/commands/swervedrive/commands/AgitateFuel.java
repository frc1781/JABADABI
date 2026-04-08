package frc.robot.commands.swervedrive.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Deploy;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;


public class AgitateFuel extends Command {
    Deploy deploy;
    Intake intake;
    Timer agitateTime;
    double collectorTarget;
    double intakeMotorPower;
    double agitateHighPoint;

    public AgitateFuel(Deploy deploy, Intake intake) {
        this.deploy = deploy;
        this.intake = intake;
        addRequirements(deploy, intake);
    }

    @Override
    public void initialize() {
        agitateTime = new Timer();
        agitateTime.restart();
        agitateHighPoint = Constants.Deploy.AGITATE_HIGH;
        collectorTarget = Constants.Deploy.AGITATE_LOW;
        intakeMotorPower = 1;
        Logger.recordOutput(getName() + "/currentCommand", "agitateFuel");
    }

    @Override
    public void execute() {
         if (Math.abs(deploy.getAbsoluteEncoder().getPosition() - collectorTarget) < 0.04 || agitateTime.hasElapsed(Constants.Deploy.AGITATE_PERIOD)) {
          if (collectorTarget == Constants.Deploy.AGITATE_HIGH) {
            agitateHighPoint = deploy.getAbsoluteEncoder().getPosition(); //may not have gone in all the way it wanted to
            //so, go back the difference between high and low, but not farther than collect set point, which is as far as the collector can go
            collectorTarget = Math.max(Constants.Deploy.COLLECT_SET_POINT, agitateHighPoint - (Constants.Deploy.AGITATE_HIGH - Constants.Deploy.AGITATE_LOW));
          }
          else {
            collectorTarget = Constants.Deploy.AGITATE_HIGH;
          }
          agitateTime.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collectorTarget = Constants.Deploy.COLLECT_SET_POINT;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}