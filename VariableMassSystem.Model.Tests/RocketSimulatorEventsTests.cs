using VariableMassSystem.Model;

namespace VariableMassSystem.Model.Tests;

public class RocketSimulatorEventsTests
{
    private static RocketParameters VerticalVacuumParameters => new()
    {
        M0 = 1000,
        MDry = 400,
        MassFlowRate = 10,
        ExhaustVelocity = 3000,
        LaunchAngleDeg = 90,
        TimeStep = 0.05,
        G = 9.81,
        DragCoefficient = 0,
        MaxTime = 700
    };

    [Fact]
    public void VerticalVacuum_RefinesBurnoutApogeeAndImpactEvents()
    {
        var result = new RocketSimulator().Simulate(VerticalVacuumParameters);

        Assert.Equal(EndReason.Impact, result.EndReason);
        Assert.NotNull(result.BurnoutPoint);
        Assert.NotNull(result.ApogeePoint);
        Assert.NotNull(result.ImpactPoint);

        var burnout = result.BurnoutPoint!.Value;
        var apogee = result.ApogeePoint!.Value;
        var impact = result.ImpactPoint!.Value;
        var expectedApogeeTime = burnout.T + burnout.Vy / VerticalVacuumParameters.G;
        var expectedApogeeHeight = burnout.Y +
            burnout.Vy * burnout.Vy / (2 * VerticalVacuumParameters.G);

        Assert.Equal(VerticalVacuumParameters.BurnTime, burnout.T);
        Assert.Equal(VerticalVacuumParameters.MDry, burnout.Mass);
        Assert.Equal(
            VerticalVacuumParameters.MassFlowRate * VerticalVacuumParameters.ExhaustVelocity,
            burnout.ThrustY,
            8);
        Assert.InRange(Math.Abs(apogee.T - expectedApogeeTime), 0, 1e-8);
        Assert.InRange(Math.Abs(apogee.Y - expectedApogeeHeight), 0, 1e-7);
        Assert.InRange(Math.Abs(apogee.Vy), 0, 1e-12);
        Assert.Equal(0, impact.Y);
        Assert.Equal(impact, result.Points[^1]);
        Assert.Equal(impact.T, result.FlightTime);
    }

    [Fact]
    public void ZeroGravity_ReachesMaxTimeAndIncludesExactBurnoutPoint()
    {
        var parameters = VerticalVacuumParameters with
        {
            G = 0,
            TimeStep = 0.07,
            MaxTime = 61
        };

        var result = new RocketSimulator().Simulate(parameters);

        Assert.Equal(EndReason.MaxTimeReached, result.EndReason);
        Assert.Equal(parameters.MaxTime, result.Points[^1].T);
        Assert.NotNull(result.BurnoutPoint);
        Assert.Equal(parameters.BurnTime, result.BurnoutPoint!.Value.T);
    }

    [Fact]
    public void Simulate_RejectsInvalidParameters()
    {
        var parameters = VerticalVacuumParameters with { TimeStep = 0 };

        Assert.Throws<ArgumentException>(() => new RocketSimulator().Simulate(parameters));
    }

    [Theory]
    [InlineData(ThrustMode.FixedAngle)]
    [InlineData(ThrustMode.VelocityAligned)]
    public void ZeroGravityAndDrag_KeepTrajectoryOnLaunchRay(ThrustMode thrustMode)
    {
        const double launchAngle = 45;
        var parameters = VerticalVacuumParameters with
        {
            LaunchAngleDeg = launchAngle,
            ThrustMode = thrustMode,
            G = 0,
            DragCoefficient = 0,
            TimeStep = 0.07,
            MaxTime = 61
        };
        var result = new RocketSimulator().Simulate(parameters);

        foreach (var point in result.Points.Where(point => point.T > 0))
        {
            var actualAngle = Math.Atan2(point.Y, point.X) * 180 / Math.PI;
            Assert.InRange(Math.Abs(actualAngle - launchAngle), 0, 1e-10);
        }
    }

    [Fact]
    public void TrajectoryTimesIncreaseAndMassNeverIncreases()
    {
        var result = new RocketSimulator().Simulate(VerticalVacuumParameters with { MaxTime = 100 });

        for (var index = 1; index < result.Points.Count; index++)
        {
            Assert.True(result.Points[index].T > result.Points[index - 1].T);
            Assert.True(result.Points[index].Mass <= result.Points[index - 1].Mass);
        }

        Assert.Equal(VerticalVacuumParameters.MDry, result.BurnoutPoint!.Value.Mass);
    }
}
