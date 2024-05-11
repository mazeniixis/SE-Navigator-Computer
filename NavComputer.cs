// Navigator Computer class
public class NavComputer
{
    readonly IMyShipController _refBlock;
    readonly List<IMyGyro> myGyros = new List<IMyGyro>();
    readonly PID _pitchPID, _yawPID;

    Vector3D _rotationPYR = Vector3D.Zero, _forwardVector = Vector3D.Zero, _upVector = Vector3D.Zero, _rotSpeedPYR = Vector3D.Zero;
    double _slowdownAngle, _updatesPerSecond;
    public AlignMode AlignMode = AlignMode.None;
    NavStatus _navStatus = NavStatus.Off;

    int precision = 6;

    // thrusters list
    readonly List<IMyThrust> _forwardThrusts = new List<IMyThrust>();
    readonly List<IMyThrust> _backwardThrusts = new List<IMyThrust>();
    readonly List<IMyThrust> _leftThrusts = new List<IMyThrust>();
    readonly List<IMyThrust> _rightThrusts = new List<IMyThrust>();
    readonly List<IMyThrust> _upThrusts = new List<IMyThrust>();
    readonly List<IMyThrust> _downThrusts = new List<IMyThrust>();

    Action<string> Debug;

    public NavComputer(IMyShipController refBlock, PID pitchPID, PID yawPID, double updatesPerSecond, double slowdownAngle, Action<string> debug)
    {
        _refBlock = refBlock;
        _pitchPID = pitchPID;
        _yawPID = yawPID;
        _updatesPerSecond = updatesPerSecond;
        _slowdownAngle = slowdownAngle;
        Debug = debug;
    }

    public void AddGyro(IMyGyro gyro)
    {
        if (gyro != null)
            myGyros.Add(gyro);
    }

    public void AddGyros(List<IMyGyro> gyros) => myGyros.AddRange(gyros);

    public void AddThrust(IMyThrust thrust)
    {
        if (thrust == null) return;
        var refOrient = _refBlock.WorldMatrix.GetOrientation();
        var thrustForward = thrust.WorldMatrix.GetOrientation().Forward;

        if (refOrient.Forward == thrustForward) _forwardThrusts.Add(thrust);
        else if (refOrient.Backward == thrustForward) _backwardThrusts.Add(thrust);
        else if (refOrient.Left == thrustForward) _leftThrusts.Add(thrust);
        else if (refOrient.Right == thrustForward) _rightThrusts.Add(thrust);
        else if (refOrient.Up == thrustForward) _upThrusts.Add(thrust);
        else if (refOrient.Down == thrustForward) _downThrusts.Add(thrust);
    }

    public void TestThruster(ThrustDirection direction, float power)
    {
        power = MathHelper.Clamp(power, 0, 100) / 100;
        List<IMyThrust> tempThrust;
        if (direction == ThrustDirection.Forward) tempThrust = _forwardThrusts;
        else if (direction == ThrustDirection.Backward) tempThrust = _backwardThrusts;
        else if (direction == ThrustDirection.Left) tempThrust = _leftThrusts;
        else if (direction == ThrustDirection.Right) tempThrust = _rightThrusts;
        else if (direction == ThrustDirection.Up) tempThrust = _upThrusts;
        else tempThrust = _downThrusts;

        foreach (var t in tempThrust)
            t.ThrustOverride = power * t.MaxThrust;
    }

    public bool AlignToHorizon()
    {
        if (Vector3D.IsZero(_upVector))
            return false;
        SetForwardVector(_refBlock.WorldMatrix.Left.Cross(_upVector));
        return true;
    }

    public void SetForwardVector(Vector3D targetPos) => _forwardVector = targetPos;

    void GetRotationPYR()
    {
        var transposedWm = MatrixD.Transpose(_refBlock.WorldMatrix);
        var forwardVect = Vector3D.Rotate(VectorMath.SafeNormalize(_forwardVector), transposedWm);

        Vector3D axis, leftVector = Vector3D.Zero;
        double angle;

        if (!Vector3D.IsZero(_upVector))
            leftVector = Vector3D.Cross(Vector3D.Rotate(_upVector, transposedWm), forwardVect);

        if (Vector3D.IsZero(_upVector) || Vector3D.IsZero(leftVector))
        {
            axis = new Vector3D(-forwardVect.Y, forwardVect.X, 0);
            angle = Math.Acos(MathHelper.Clamp(-forwardVect.Z, -1.0, 1.0));
        }
        else
        {
            leftVector = VectorMath.SafeNormalize(leftVector);
            var upVector = Vector3D.Cross(forwardVect, leftVector);
            var newMatrix = new MatrixD()
            {
                Forward = forwardVect,
                Left = leftVector,
                Up = upVector
            };

            axis = new Vector3D(newMatrix.M32 - newMatrix.M23, newMatrix.M13 - newMatrix.M31, newMatrix.M21 - newMatrix.M12);
            double trace = newMatrix.M11 + newMatrix.M22 + newMatrix.M33;
            angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1, 1));
        }

        if (Vector3D.IsZero(axis))
        {
            angle = forwardVect.Z < 0 ? 0 : Math.PI; // THIS IS THE PROBLEM!
            _rotationPYR = new Vector3D(0, angle, 0);
        }
        else
            _rotationPYR = VectorMath.SafeNormalize(axis) * angle;
    }

    void AngleController()
    {
        if (_pitchPID != null) _rotSpeedPYR.X = _pitchPID.Control(_rotationPYR.X);
        if (_yawPID != null) _rotSpeedPYR.Y = _yawPID.Control(_rotationPYR.Y);

        if (Math.Abs(_rotationPYR.X) < _slowdownAngle) _rotSpeedPYR.X = _updatesPerSecond * .5 * _rotationPYR.X;
        if (Math.Abs(_rotationPYR.Y) < _slowdownAngle) _rotSpeedPYR.Y = _updatesPerSecond * .5 * _rotationPYR.Y;
    }

    void ApplyGyroOverride(bool bOverride = true)
    {
        var worldRotationPYR = Vector3D.TransformNormal(_rotSpeedPYR, _refBlock.WorldMatrix);

        Debug("We are here!");

        foreach (var gyro in myGyros)
        {
            if (!bOverride)
            {
                gyro.GyroOverride = false;
                continue;
            }

            var gyroPYR = Vector3D.TransformNormal(worldRotationPYR, Matrix.Transpose(gyro.WorldMatrix));

            gyro.Pitch = (float)gyroPYR.X;
            gyro.Yaw = (float)gyroPYR.Y;
            gyro.Roll = (float)gyroPYR.Z;
            gyro.GyroOverride = true;
        }
    }

    void Update()
    {
        if ((AlignMode & AlignMode.Natural) != 0) _upVector = -_refBlock.GetNaturalGravity();
        else if ((AlignMode & AlignMode.Artificial) != 0) _upVector = -_refBlock.GetArtificialGravity();
        else if ((AlignMode & AlignMode.Total) != 0) _upVector = -_refBlock.GetTotalGravity();
        else if ((AlignMode & AlignMode.Target) == 0) _upVector = Vector3D.Zero;
    }

    public void Control()
    {
        if (_navStatus == NavStatus.Off)
            return;
        Update();
        GetRotationPYR();
        AngleController();
        ApplyGyroOverride();
    }

    public void ReadData()
    {
        // this is to read the data that I otherwise cannot
        Debug($"Forward Vector: {Vector3D.Round(_forwardVector, precision)}");
        Debug($"Rotation PYR: {Vector3D.Round(_rotationPYR, precision)}");
        Debug($"Rotation Speed PYR: {Vector3D.Round(_rotSpeedPYR, precision)}");
    }

    public void SetStatus(NavStatus navStatus)
    {
        if (navStatus == NavStatus.Off) ApplyGyroOverride(false);
        _navStatus = navStatus;
    }
}
