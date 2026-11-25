using VariableMassSystem.Model;

namespace VariableMassSystem.Model.Tests;

public class RocketSimulatorVerificationTests
{
    [Theory]
    [InlineData(45, ThrustMode.FixedAngle, 0.01)]
    [InlineData(90, ThrustMode.FixedAngle, 0.01)]
    [InlineData(45, ThrustMode.VelocityAligned, 0.01)]
    [InlineData(90, ThrustMode.VelocityAligned, 0.01)]
    [InlineData(45, ThrustMode.FixedAngle, 0.07)]
    [InlineData(90, ThrustMode.FixedAngle, 0.07)]
    [InlineData(45, ThrustMode.VelocityAligned, 0.07)]
    [InlineData(90, ThrustMode.VelocityAligned, 0.07)]
    public void VacuumBurnoutSpeed_MatchesTsiolkovskyEquation(
        double launchAngle,
        ThrustMode thrustMode,
        double timeStep)
    {
        var parameters = CreateParameters() with
        {
            LaunchAngleDeg = launchAngle,
            ThrustMode = thrustMode,
            TimeStep = timeStep,
            G = 0,
            DragCoefficient = 0,
            MaxTime = 61
        };

        var burnout = new RocketSimulator().Simulate(parameters).BurnoutPoint!.Value;
        var expectedSpeed = parameters.ExhaustVelocity * Math.Log(parameters.M0 / parameters.MDry);
        var relativeError = Math.Abs(burnout.Speed - expectedSpeed) / expectedSpeed;

        Assert.InRange(relativeError, 0, 1e-9);
    }

    [Fact]
    public void VerticalVacuumBurnoutState_MatchesAnalyticSolution()
    {
        var parameters = CreateParameters() with { TimeStep = 0.01, MaxTime = 61 };
        var burnout = new RocketSimulator().Simulate(parameters).BurnoutPoint!.Value;
        var burnTime = parameters.BurnTime;
        var massFractionBurned = parameters.MassFlowRate * burnTime / parameters.M0;
        var expectedVy = parameters.ExhaustVelocity * Math.Log(parameters.M0 / parameters.MDry) -
            parameters.G * burnTime;
        var expectedY = parameters.ExhaustVelocity * parameters.M0 / parameters.MassFlowRate *
            ((1 - massFractionBurned) * Math.Log(1 - massFractionBurned) + massFractionBurned) -
            parameters.G * burnTime * burnTime / 2;

        AssertRelativeClose(burnout.Vy, expectedVy, 1e-9);
        AssertRelativeClose(burnout.Y, expectedY, 1e-9);
    }

    [Fact]
    public void VacuumCoast_PreservesHorizontalVelocityAndMechanicalEnergy()
    {
        var parameters = CreateParameters() with { TimeStep = 0.05, MaxTime = 100 };
        var result = new RocketSimulator().Simulate(parameters);
        var burnout = result.BurnoutPoint!.Value;
        var coastPoints = result.Points.Where(point => point.T > burnout.T).ToArray();
        var initialEnergy = MechanicalEnergy(burnout, parameters);

        Assert.NotEmpty(coastPoints);
        foreach (var point in coastPoints)
        {
            Assert.InRange(Math.Abs(point.Vx - burnout.Vx), 0, 1e-10);
            AssertRelativeClose(MechanicalEnergy(point, parameters), initialEnergy, 1e-8);
        }
    }

    private static RocketParameters CreateParameters() => new()
    {
        M0 = 1000,
        MDry = 400,
        MassFlowRate = 10,
        ExhaustVelocity = 3000,
        LaunchAngleDeg = 90,
        TimeStep = 0.01,
        G = 9.81,
        DragCoefficient = 0,
        MaxTime = 61
    };

    private static double MechanicalEnergy(TrajectoryPoint point, RocketParameters parameters) =>
        0.5 * point.Mass * point.Speed * point.Speed +
        point.Mass * parameters.G * point.Y;

    private static void AssertRelativeClose(double actual, double expected, double tolerance)
    {
        Assert.InRange(Math.Abs(actual - expected) / Math.Abs(expected), 0, tolerance);
    }
}
