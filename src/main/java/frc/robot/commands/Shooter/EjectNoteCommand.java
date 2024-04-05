// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.Indexer.NudgeIndexer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectNoteCommand extends SequentialCommandGroup {
  /** Creates a new EjectNoteCommand. */
  public EjectNoteCommand(Shooter shooter, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinFlywheelsCommand(shooter, 100),
      new WaitCommand(0.1),
      new WaitUntilCommand(() -> shooter.atTargetSpeed(Constants.kShooter.FLYWHEEL_RPM_ACCEPTABLE_ERROR)),
      new NudgeIndexer(indexer)
    );
  }
}
