package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCL4OuttakeCmd extends Command {

private final IntakeSys intake;   

public IntakeCL4OuttakeCmd(IntakeSys intake){
    this.intake = intake;

    addRequirements(intake);
}

@Override
public void initialize() {
    intake.setIsOuttakingCL4(true);
}

@Override
public void execute(){

}

@Override
public void end(boolean interrupted) {

}

@Override
public boolean isFinished(){
    return true;
}
}
 