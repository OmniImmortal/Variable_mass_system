namespace VariableMassSystem.Model;

/// <summary>
/// Параметры ракеты и окружающей среды для расчёта траектории.
/// </summary>
public sealed record RocketParameters
{
    public const string InsufficientThrustMessage = "Недостаточная тяга для отрыва от стартового стола";

    public double M0 { get; init; }
    public double MDry { get; init; }
    public double MassFlowRate { get; init; }
    public double ExhaustVelocity { get; init; }
    public double LaunchAngleDeg { get; init; }
    public double TimeStep { get; init; }
    public double G { get; init; } = 9.81;
    public double DragCoefficient { get; init; } = 0.45;
    public double CrossSectionArea { get; init; } = 0.5;
    public double AirDensity { get; init; } = 1.225;
    public ThrustMode ThrustMode { get; init; } = ThrustMode.FixedAngle;
    public double MaxTime { get; init; } = 3600;

    public double BurnTime => (M0 - MDry) / MassFlowRate;
    public double FuelMass => M0 - MDry;

    public bool CanLiftOff =>
        G == 0 ||
        ExhaustVelocity * MassFlowRate * Math.Sin(LaunchAngleDeg * Math.PI / 180) > M0 * G;

    /// <summary>
    /// Возвращает все ошибки валидации параметров.
    /// </summary>
    public IReadOnlyList<string> Validate()
    {
        var errors = new List<string>();

        AddFiniteError(errors, nameof(M0), M0);
        AddFiniteError(errors, nameof(MDry), MDry);
        AddFiniteError(errors, nameof(MassFlowRate), MassFlowRate);
        AddFiniteError(errors, nameof(ExhaustVelocity), ExhaustVelocity);
        AddFiniteError(errors, nameof(LaunchAngleDeg), LaunchAngleDeg);
        AddFiniteError(errors, nameof(TimeStep), TimeStep);
        AddFiniteError(errors, nameof(G), G);
        AddFiniteError(errors, nameof(DragCoefficient), DragCoefficient);
        AddFiniteError(errors, nameof(CrossSectionArea), CrossSectionArea);
        AddFiniteError(errors, nameof(AirDensity), AirDensity);
        AddFiniteError(errors, nameof(MaxTime), MaxTime);

        if (double.IsFinite(M0) && double.IsFinite(MDry) && !(M0 > MDry && MDry > 0))
        {
            errors.Add("Массы должны удовлетворять условию M0 > MDry > 0.");
        }

        AddPositiveError(errors, nameof(MassFlowRate), MassFlowRate);
        AddPositiveError(errors, nameof(ExhaustVelocity), ExhaustVelocity);
        if (double.IsFinite(LaunchAngleDeg) && !(LaunchAngleDeg > 0 && LaunchAngleDeg <= 90))
        {
            errors.Add("Угол запуска должен быть больше 0 и не превышать 90 градусов.");
        }

        AddPositiveError(errors, nameof(TimeStep), TimeStep);
        AddNonNegativeError(errors, nameof(G), G);
        AddNonNegativeError(errors, nameof(DragCoefficient), DragCoefficient);
        AddNonNegativeError(errors, nameof(CrossSectionArea), CrossSectionArea);
        AddNonNegativeError(errors, nameof(AirDensity), AirDensity);
        AddPositiveError(errors, nameof(MaxTime), MaxTime);

        if (!Enum.IsDefined(ThrustMode))
        {
            errors.Add("Неизвестный режим тяги.");
        }

        if (double.IsFinite(G) &&
            G > 0 &&
            double.IsFinite(M0) &&
            double.IsFinite(MassFlowRate) &&
            double.IsFinite(ExhaustVelocity) &&
            double.IsFinite(LaunchAngleDeg) &&
            M0 > 0 &&
            MassFlowRate > 0 &&
            ExhaustVelocity > 0 &&
            LaunchAngleDeg > 0 &&
            LaunchAngleDeg <= 90 &&
            !CanLiftOff)
        {
            errors.Add(InsufficientThrustMessage);
        }

        return errors;
    }

    /// <summary>
    /// Проверяет параметры и выбрасывает исключение при наличии ошибок.
    /// </summary>
    public void EnsureValid()
    {
        var errors = Validate();
        if (errors.Count > 0)
        {
            throw new ArgumentException(string.Join(Environment.NewLine, errors), nameof(RocketParameters));
        }
    }

    private static void AddFiniteError(List<string> errors, string name, double value)
    {
        if (!double.IsFinite(value))
        {
            errors.Add($"Параметр {name} должен быть конечным числом.");
        }
    }

    private static void AddPositiveError(List<string> errors, string name, double value)
    {
        if (double.IsFinite(value) && value <= 0)
        {
            errors.Add($"Параметр {name} должен быть больше нуля.");
        }
    }

    private static void AddNonNegativeError(List<string> errors, string name, double value)
    {
        if (double.IsFinite(value) && value < 0)
        {
            errors.Add($"Параметр {name} не может быть отрицательным.");
        }
    }
}
