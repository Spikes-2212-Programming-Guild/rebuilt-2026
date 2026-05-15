package frc.robot.oi;

public record Actions(
        Runnable toggleSpeedScale,
        Runnable toggleFieldRelative,
        Runnable toggleSquareInputs,
        Runnable toggleDeadband,
        Runnable toggleSlewRateLimiter
) {
}
