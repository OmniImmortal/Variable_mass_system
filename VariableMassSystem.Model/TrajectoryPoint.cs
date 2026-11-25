namespace VariableMassSystem.Model;

/// <summary>
/// Состояние ракеты и действующие на неё силы в один момент времени.
/// </summary>
public readonly record struct TrajectoryPoint(
    double T,
    double X,
    double Y,
    double Vx,
    double Vy,
    double Ax,
    double Ay,
    double Mass,
    double ThrustX,
    double ThrustY,
    double DragX,
    double DragY)
{
    public double Speed => Math.Sqrt(Vx * Vx + Vy * Vy);

    public double Acceleration => Math.Sqrt(Ax * Ax + Ay * Ay);
}
