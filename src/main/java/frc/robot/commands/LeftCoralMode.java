package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class LeftCoralMode extends Command {

    public LeftCoralMode() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.leftCoralMode = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 