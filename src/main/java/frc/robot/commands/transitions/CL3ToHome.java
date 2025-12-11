package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderCL23BufferCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotCL23Cmd;
import frc.robot.commands.extender.ExtenderCL3FinalBufferCmd;
import frc.robot.commands.elevator.ElevatorCL3BufferCmd;

import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL3ToHome extends SequentialCommandGroup {

  public CL3ToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateHOME(),
      new PivotCL23Cmd(pivot),
      new WaitCommand(0.2),
      new ElevatorCL3BufferCmd(elevator),
      new ExtenderCL23BufferCmd(extender),
      new WaitCommand(0.2),
      new ExtenderCL3FinalBufferCmd(extender),
      new WaitCommand(0.5),
      new PivotHomeCmd(pivot),
      new WaitCommand(1.0),
      new ExtenderHomeCmd(extender),
      new ElevatorHomeCmd(elevator)
      
    );
  }
}