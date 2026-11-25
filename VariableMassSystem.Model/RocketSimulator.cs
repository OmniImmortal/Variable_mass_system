namespace VariableMassSystem.Model;

/// <summary>
/// Выполняет численное моделирование движения ракеты.
/// </summary>
public sealed class RocketSimulator
{
    public SimulationResult Simulate(RocketParameters parameters)
    {
        ArgumentNullException.ThrowIfNull(parameters);
        parameters.EnsureValid();

        var points = new List<TrajectoryPoint>();
        var state = new State(0, 0, 0, 0);
        var time = 0d;
        var initialPoint = CreatePoint(state, time, parameters, activeThrust: true, isBurnout: false);
        points.Add(initialPoint);

        TrajectoryPoint? burnoutPoint = null;
        while (time < parameters.MaxTime)
        {
            var activeThrust = time < parameters.BurnTime;
            var step = Math.Min(parameters.TimeStep, parameters.MaxTime - time);
            var reachesBurnout = activeThrust && time + step >= parameters.BurnTime;
            if (reachesBurnout)
            {
                step = parameters.BurnTime - time;
            }

            var nextState = Rk4Step(state, time, step, parameters, activeThrust);
            var nextTime = reachesBurnout
                ? parameters.BurnTime
                : time + step >= parameters.MaxTime
                    ? parameters.MaxTime
                    : time + step;
            var point = CreatePoint(nextState, nextTime, parameters, activeThrust, reachesBurnout);

            points.Add(point);
            state = nextState;
            time = nextTime;

            if (reachesBurnout)
            {
                // Точка выгорания относится к концу активной фазы для корректного отображения графиков.
                burnoutPoint = point;
            }
        }

        return BuildResult(
            points,
            burnoutPoint,
            apogeePoint: null,
            impactPoint: null,
            EndReason.MaxTimeReached);
    }

    private static SimulationResult BuildResult(
        List<TrajectoryPoint> points,
        TrajectoryPoint? burnoutPoint,
        TrajectoryPoint? apogeePoint,
        TrajectoryPoint? impactPoint,
        EndReason endReason)
    {
        var maxSpeed = 0d;
        var maxAltitude = double.NegativeInfinity;
        foreach (var point in points)
        {
            maxSpeed = Math.Max(maxSpeed, point.Speed);
            maxAltitude = Math.Max(maxAltitude, point.Y);
        }

        var lastPoint = points[^1];
        return new SimulationResult(
            points.AsReadOnly(),
            burnoutPoint,
            apogeePoint,
            impactPoint,
            maxSpeed,
            maxAltitude,
            lastPoint.T,
            lastPoint.X,
            endReason);
    }

    private static TrajectoryPoint CreatePoint(
        State state,
        double time,
        RocketParameters parameters,
        bool activeThrust,
        bool isBurnout)
    {
        var mass = activeThrust
            ? isBurnout
                ? parameters.MDry
                : Math.Max(parameters.MDry, parameters.M0 - parameters.MassFlowRate * time)
            : parameters.MDry;
        var forces = EvaluateForces(state, mass, parameters, activeThrust);
        return new TrajectoryPoint(
            time,
            state.X,
            state.Y,
            state.Vx,
            state.Vy,
            forces.Ax,
            forces.Ay,
            mass,
            forces.ThrustX,
            forces.ThrustY,
            forces.DragX,
            forces.DragY);
    }

    private static State Rk4Step(State state, double time, double step, RocketParameters parameters, bool activeThrust)
    {
        var k1 = Derivative(state, time, parameters, activeThrust);
        var k2 = Derivative(state + k1 * (step / 2), time + step / 2, parameters, activeThrust);
        var k3 = Derivative(state + k2 * (step / 2), time + step / 2, parameters, activeThrust);
        var k4 = Derivative(state + k3 * step, time + step, parameters, activeThrust);
        return state + (k1 + k2 * 2 + k3 * 2 + k4) * (step / 6);
    }

    private static State Derivative(State state, double time, RocketParameters parameters, bool activeThrust)
    {
        var mass = activeThrust
            ? Math.Max(parameters.MDry, parameters.M0 - parameters.MassFlowRate * time)
            : parameters.MDry;
        var forces = EvaluateForces(state, mass, parameters, activeThrust);
        return new State(state.Vx, state.Vy, forces.Ax, forces.Ay);
    }

    private static Forces EvaluateForces(State state, double mass, RocketParameters parameters, bool activeThrust)
    {
        var speed = Math.Sqrt(state.Vx * state.Vx + state.Vy * state.Vy);
        var dragScale = -0.5 * parameters.AirDensity * parameters.DragCoefficient *
            parameters.CrossSectionArea * speed;
        var dragX = dragScale * state.Vx;
        var dragY = dragScale * state.Vy;

        var thrustX = 0d;
        var thrustY = 0d;
        if (activeThrust)
        {
            var angle = parameters.LaunchAngleDeg * Math.PI / 180;
            if (parameters.ThrustMode == ThrustMode.VelocityAligned && speed >= 1e-9)
            {
                thrustX = parameters.MassFlowRate * parameters.ExhaustVelocity * state.Vx / speed;
                thrustY = parameters.MassFlowRate * parameters.ExhaustVelocity * state.Vy / speed;
            }
            else
            {
                thrustX = parameters.MassFlowRate * parameters.ExhaustVelocity * Math.Cos(angle);
                thrustY = parameters.MassFlowRate * parameters.ExhaustVelocity * Math.Sin(angle);
            }
        }

        var ax = (dragX + thrustX) / mass;
        var ay = -parameters.G + (dragY + thrustY) / mass;
        return new Forces(ax, ay, thrustX, thrustY, dragX, dragY);
    }

    private readonly record struct State(double X, double Y, double Vx, double Vy)
    {
        public static State operator +(State left, State right) =>
            new(left.X + right.X, left.Y + right.Y, left.Vx + right.Vx, left.Vy + right.Vy);

        public static State operator *(State state, double factor) =>
            new(state.X * factor, state.Y * factor, state.Vx * factor, state.Vy * factor);
    }

    private readonly record struct Forces(double Ax, double Ay, double ThrustX, double ThrustY, double DragX, double DragY);
}
