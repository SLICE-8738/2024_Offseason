// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import frc.robot.commands.Drivetrain.AlignWithNoteCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndStoreNote extends ParallelDeadlineGroup {
  /** Creates a new AlignAndStoreNote. */
  public AlignAndStoreNote(Drivetrain drivetrain, Indexer indexer, Intake intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new StoreNote(indexer, intake), new AlignWithNoteCommand(drivetrain));
  }
}
