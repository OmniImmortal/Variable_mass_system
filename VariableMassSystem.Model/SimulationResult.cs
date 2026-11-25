namespace VariableMassSystem.Model;

/// <summary>
/// Результат моделирования траектории ракеты.
/// </summary>
public sealed record SimulationResult(
    IReadOnlyList<TrajectoryPoint> Points,
    TrajectoryPoint? BurnoutPoint,
    TrajectoryPoint? ApogeePoint,
    TrajectoryPoint? ImpactPoint,
    double MaxSpeed,
    double MaxAltitude,
    double FlightTime,
    double Range,
    EndReason EndReason);
