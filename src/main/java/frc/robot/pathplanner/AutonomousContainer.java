package frc.robot.pathplanner;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//IT'S A PLACEHOLDER DON'T CR THIS!!!!!
public class AutonomousContainer {

    public static final ProfiledPIDController ROTATIONAL_PID_CONTROLLER =
            new ProfiledPIDController(-1, -1, -1, new TrapezoidProfile.Constraints(
                    -1, -1));
}
