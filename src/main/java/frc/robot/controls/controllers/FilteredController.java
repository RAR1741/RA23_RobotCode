package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controls.Deadband;
import frc.robot.controls.SquaredInput;

public class FilteredController extends XboxController {
  private static final double DEADBAND_LIMIT = 0.01;

  private boolean useDeadband;
  private boolean useSquaredInput;

  private Deadband deadband = new Deadband(DEADBAND_LIMIT);
  private SquaredInput squaredInput = new SquaredInput(DEADBAND_LIMIT);

  public FilteredController(int port) {
    super(port);
    useDeadband = false;
    useSquaredInput = false;
  }

  public FilteredController(int port, boolean useDeadband, boolean useSquaredInput) {
    this(port);
    this.useDeadband = useDeadband;
    this.useSquaredInput = useSquaredInput;
  }

  public double getFilteredAxis(int axis) {
    double value = this.getRawAxis(axis);

    // Apply squared input, if requested
    if (useSquaredInput) {
      value = squaredInput.scale(value);
    }

    // Apply deadband, if requested
    if (useDeadband) {
      value = deadband.scale(value);
    }

    return value;
  }
}
