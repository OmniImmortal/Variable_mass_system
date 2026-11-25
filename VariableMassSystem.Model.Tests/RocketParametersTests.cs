using VariableMassSystem.Model;

namespace VariableMassSystem.Model.Tests;

public class RocketParametersTests
{
    private static RocketParameters ValidParameters => new()
    {
        M0 = 1000,
        MDry = 400,
        MassFlowRate = 10,
        ExhaustVelocity = 3000,
        LaunchAngleDeg = 90,
        TimeStep = 0.01
    };

    [Fact]
    public void ValidParameters_HaveNoErrorsAndExposeDerivedValues()
    {
        var parameters = ValidParameters;

        Assert.Empty(parameters.Validate());
        Assert.Equal(60, parameters.BurnTime);
        Assert.Equal(600, parameters.FuelMass);
        Assert.True(parameters.CanLiftOff);
    }

    [Fact]
    public void Validate_ReportsInvalidMasses()
    {
        Assert.NotEmpty((ValidParameters with { M0 = 400 }).Validate());
        Assert.NotEmpty((ValidParameters with { MDry = 1000 }).Validate());
        Assert.NotEmpty((ValidParameters with { MDry = 0 }).Validate());
    }

    [Fact]
    public void Validate_AcceptsZeroForNonNegativeParameters()
    {
        var parameters = ValidParameters with
        {
            G = 0,
            DragCoefficient = 0,
            CrossSectionArea = 0,
            AirDensity = 0
        };

        Assert.Empty(parameters.Validate());
    }

    [Theory]
    [InlineData(nameof(RocketParameters.MassFlowRate))]
    [InlineData(nameof(RocketParameters.ExhaustVelocity))]
    [InlineData(nameof(RocketParameters.TimeStep))]
    [InlineData(nameof(RocketParameters.MaxTime))]
    public void Validate_RequiresPositiveValues(string property)
    {
        var parameters = property switch
        {
            nameof(RocketParameters.MassFlowRate) => ValidParameters with { MassFlowRate = 0 },
            nameof(RocketParameters.ExhaustVelocity) => ValidParameters with { ExhaustVelocity = 0 },
            nameof(RocketParameters.TimeStep) => ValidParameters with { TimeStep = 0 },
            nameof(RocketParameters.MaxTime) => ValidParameters with { MaxTime = 0 },
            _ => throw new ArgumentOutOfRangeException(nameof(property))
        };

        Assert.NotEmpty(parameters.Validate());
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    [InlineData(90.001)]
    public void Validate_RejectsLaunchAnglesOutsideAllowedRange(double angle)
    {
        Assert.NotEmpty((ValidParameters with { LaunchAngleDeg = angle }).Validate());
    }

    [Theory]
    [InlineData(-1, nameof(RocketParameters.G))]
    [InlineData(-1, nameof(RocketParameters.DragCoefficient))]
    [InlineData(-1, nameof(RocketParameters.CrossSectionArea))]
    [InlineData(-1, nameof(RocketParameters.AirDensity))]
    public void Validate_RejectsNegativeEnvironmentalParameters(double value, string property)
    {
        var parameters = property switch
        {
            nameof(RocketParameters.G) => ValidParameters with { G = value },
            nameof(RocketParameters.DragCoefficient) => ValidParameters with { DragCoefficient = value },
            nameof(RocketParameters.CrossSectionArea) => ValidParameters with { CrossSectionArea = value },
            nameof(RocketParameters.AirDensity) => ValidParameters with { AirDensity = value },
            _ => throw new ArgumentOutOfRangeException(nameof(property))
        };

        Assert.NotEmpty(parameters.Validate());
    }

    [Fact]
    public void Validate_ReportsEveryNonFiniteNumericParameter()
    {
        var parameters = ValidParameters with
        {
            M0 = double.NaN,
            MDry = double.PositiveInfinity,
            MassFlowRate = double.NegativeInfinity,
            ExhaustVelocity = double.NaN,
            LaunchAngleDeg = double.PositiveInfinity,
            TimeStep = double.NaN,
            G = double.PositiveInfinity,
            DragCoefficient = double.NegativeInfinity,
            CrossSectionArea = double.NaN,
            AirDensity = double.PositiveInfinity,
            MaxTime = double.NaN
        };

        Assert.Equal(11, parameters.Validate().Count);
    }

    [Fact]
    public void Validate_UsesExactInsufficientThrustMessage()
    {
        var parameters = ValidParameters with { ExhaustVelocity = 100, MassFlowRate = 1 };

        Assert.Contains(RocketParameters.InsufficientThrustMessage, parameters.Validate());
        Assert.False(parameters.CanLiftOff);
    }

    [Fact]
    public void Validate_RejectsUnknownThrustMode()
    {
        Assert.NotEmpty((ValidParameters with { ThrustMode = (ThrustMode)123 }).Validate());
    }

    [Fact]
    public void CanLiftOff_IsTrueWhenGravityIsZero()
    {
        Assert.True((ValidParameters with { G = 0 }).CanLiftOff);
    }

    [Fact]
    public void EnsureValid_ThrowsArgumentExceptionWithAllErrors()
    {
        var parameters = ValidParameters with { M0 = 400, TimeStep = 0 };

        var exception = Assert.Throws<ArgumentException>(parameters.EnsureValid);

        Assert.Contains("M0", exception.Message);
        Assert.Contains("TimeStep", exception.Message);
    }
}
