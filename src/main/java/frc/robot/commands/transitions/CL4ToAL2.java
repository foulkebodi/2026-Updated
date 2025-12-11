package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorAL2Cmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL4ToAL2 extends SequentialCommandGroup {

  public CL4ToAL2(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new PivotGroundCmd(pivot),
      new ElevatorAL2Cmd(elevator),
      new ExtenderHomeCmd(extender)
      
      
    );
  }
}