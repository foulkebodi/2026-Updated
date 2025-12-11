package frc.robot.commands.winch;

import frc.robot.Constants.WinchConstants;
import frc.robot.subsystems.WinchSys;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchOutCmd extends Command {

    private final WinchSys winch;   

    public WinchOutCmd(WinchSys winchSys) {
        this.winch = winchSys;

        addRequirements(winchSys);
    }

    @Override
    public void initialize() {
        winch.setWinchTargetDeg(WinchConstants.outPresetDeg);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}
 