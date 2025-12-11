package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.intake.IntakeIdleCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.IntakeSys;


public class CL4BufferToHome extends SequentialCommandGroup {

  public CL4BufferToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender, IntakeSys intake) {
    super(
      new WaitCommand(0.3),
      new IntakeIdleCmd(intake),
      new ElevatorHomeCmd(elevator),
      new ExtenderHomeCmd(extender)
    );
  }
}