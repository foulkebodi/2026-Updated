package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCL4BufferCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.commands.intake.IntakeCL4OuttakeCmd;


public class CL4ToBuffer extends SequentialCommandGroup {

  public CL4ToBuffer(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender, IntakeSys intake) {
    super(
      // new StateHOME(),
      new PivotGroundCmd(pivot),
      new WaitCommand(0.5),
      new ElevatorCL4BufferCmd(elevator),
      new WaitCommand(0.2),
      new IntakeCL4OuttakeCmd(intake),
      new PivotHomeCmd(pivot)
    );
  }
}