package com.team7419.templates;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ExampleSubsystem subsystem;

  public ExampleCommand(ExampleSubsystem subsystem) {
    this.subsystem = subsystem;
    // uses addRequirements() instead of requires()
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
