    // enumerator flag that we need
    [Flags]
    public enum AlignMode { None = 0, Natural = 1, Artificial = 2, Total = 4, Target = 8 }
    public enum NavStatus { Off, On }
    public enum ThrustDirection { Forward, Backward, Left, Right, Up, Down }

    // We're borrowing a bit from Whiplash for now
    public static class VectorMath
    {
        public static Vector3D SafeNormalize(Vector3D a)
        {
            if (Vector3D.IsZero(a)) return Vector3D.Zero;
            if (Vector3D.IsUnit(ref a)) return a;
            return Vector3D.Normalize(a);
        }
    }

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

        StringBuilder debugSB = new StringBuilder();

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
                angle = forwardVect.Z < 0 ? 0 : Math.PI;
                _rotationPYR = new Vector3D(0, angle, 0);
            }
            else
                _rotationPYR = VectorMath.SafeNormalize(axis) * angle;
        }

        void AngleController()
        {
            if (_pitchPID != null) _rotSpeedPYR.X = _pitchPID.Control(_rotationPYR.X);
            if (_yawPID != null) _rotSpeedPYR.Y = _yawPID.Control(_rotationPYR.Y);
            _rotSpeedPYR.Z = _rotationPYR.Z;

            if (Math.Abs(_rotationPYR.X) < _slowdownAngle) _rotSpeedPYR.X = _updatesPerSecond * .5 * _rotationPYR.X;
            if (Math.Abs(_rotationPYR.Y) < _slowdownAngle) _rotSpeedPYR.Y = _updatesPerSecond * .5 * _rotationPYR.Y;
            if (Math.Abs(_rotSpeedPYR.Z) < _slowdownAngle) _rotSpeedPYR.Z = _updatesPerSecond * .5 * _rotationPYR.Z;
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

        public void Update()
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
            GetRotationPYR();
            AngleController();
            ApplyGyroOverride();
        }

        public void ReadData()
        {
            var surf = (_refBlock as IMyTextSurfaceProvider).GetSurface(0);
            surf.ContentType = ContentType.TEXT_AND_IMAGE;
            surf.BackgroundColor = Color.Black;
            surf.FontColor = Color.White;

            debugSB.Clear();
            debugSB.AppendLine($"Forward Vector: {Vector3D.Round(_forwardVector, precision)}");
            debugSB.AppendLine($"Rotation PYR: {Vector3D.Round(_rotationPYR, precision)}");
            debugSB.AppendLine($"Rotation Speed PYR: {Vector3D.Round(_rotSpeedPYR, precision)}");

            // this is to read the data that I otherwise cannot
            Debug(debugSB.ToString());
            surf.WriteText(debugSB);
        }

        public void SetStatus(NavStatus navStatus)
        {
            if (navStatus == NavStatus.Off) ApplyGyroOverride(false);
            _navStatus = navStatus;
        }
    }

    public class NavTestEnv : PbMAPI
    {
        NavComputer _navCom;
        MyIniKey _refKey = new MyIniKey("NAVCOMPUTER", "MAIN");
        readonly MyIni myIni = new MyIni();
        readonly MyCommandLine myCmd = new MyCommandLine();
        const double _updatesPerSecond = 10,
            _secondsPerUpdate = 1 / _updatesPerSecond,
            gyroSlowdownAngle = Math.PI / 36;

        Dictionary<string, Action<string>> _navCommands = new Dictionary<string, Action<string>>(StringComparer.OrdinalIgnoreCase);

        NavStatus _navStatus = NavStatus.Off;

        bool _bAlignToHorizon = true;

        // poor man's timer system
        int _tick = 1, _time = 13, _state = 0;

        // test thruster phase
        bool _bThrustTest = true;

        // current thruster direction
        ThrustDirection _thrustDirection;

        // Get Block function to get the blocks that is needed and initialize it
        bool GetBlock(IMyShipController t)
        {
            if (!t.IsSameConstructAs(Me) || !myIni.TryParse(t.CustomData)) return false;
            if (_navCom == null && myIni.Get(_refKey).ToBoolean())
                _navCom = new NavComputer(t, new PID(10, 0, 10, _secondsPerUpdate), new PID(10, 0, 10, _secondsPerUpdate), _secondsPerUpdate, gyroSlowdownAngle, Echo);
            return false;
        }

        void NavCommand(string cmd)
        {
            
            if (!myCmd.TryParse(cmd)) return;

            if (_navCommands.ContainsKey(myCmd.Argument(0)) && myCmd.ArgumentCount >= 2)
                _navCommands[myCmd.Argument(0)](myCmd.Argument(1));
        }

        void SetStatus(string status)
        {
            if (status.ToLower() == "on")
                _navStatus = NavStatus.On;
            else if (status.ToLower() == "off")
                _navStatus = NavStatus.Off;
            _navCom.SetStatus(_navStatus);
        }

        void SetAlignment(string align)
        {
            // switch statement
            switch(align.ToLower()) 
            {
                case "natural": _navCom.AlignMode = AlignMode.Natural;
                    break;
                case "artificial": _navCom.AlignMode = AlignMode.Artificial;
                    break;
                case "total": _navCom.AlignMode = AlignMode.Total;
                    break;
                case "target": _navCom.AlignMode = AlignMode.Target;
                    break;
                case "off": _navCom.AlignMode = AlignMode.None;
                    break;
            }
        }

        void TestThrusters()
        {
            _thrustDirection = (ThrustDirection)_state;
            _navCom.TestThruster(_thrustDirection, 100);

            ++_tick;
            if ((_tick % _time) == 0)
            {
                ++_state;
                _navCom.TestThruster(_thrustDirection, 0);
            }

            if (_state >= 6)
                _bThrustTest = false;
        }

        void Run()
        {
            if (_navStatus == NavStatus.Off) return;
            _navCom.Update();
            _navCom.AlignToHorizon();
            _navCom.Control();
            _navCom.ReadData();
        }

        public NavTestEnv(Program prog) : base(prog)
        {
            var surf = Me.GetSurface(0);
            surf.ContentType = ContentType.TEXT_AND_IMAGE;
            surf.Alignment = TextAlignment.CENTER;
            surf.BackgroundColor = Color.Black;

            List<IMyGyro> gyros = new List<IMyGyro>();
            GTS.GetBlocksOfType<IMyShipController>(null, t => GetBlock(t));
            if (_navCom == null)
            {
                surf.FontColor = Color.Red;
                surf.WriteText($"NAVCOMPUTER ERROR!\n{new string('=', 100)}\nNAV COMPUTER REF BLOCK NOT FOUND!");
                return;
            }
            GTS.GetBlocksOfType<IMyTerminalBlock>(null, t => { 
                if (!t.IsSameConstructAs(Me)) 
                    return false; 
                if (t is IMyGyro) _navCom.AddGyro(t as IMyGyro);
                if (t is IMyThrust) _navCom.AddThrust(t as IMyThrust);
                return false; });
            Runtime.UpdateFrequency = UpdateFrequency.Update10;

            _navCommands.Add("nav", SetStatus);
            _navCommands.Add("align", SetAlignment);

            surf.FontColor = Color.Green;
            surf.WriteText("NAVCOMPUTER SYSTEMS PASSED!");

            Event(Run);
            OnCommand(NavCommand);
        }
    }

    readonly NavTestEnv navTestEnv;
    
    public Program()
    {
        navTestEnv = new NavTestEnv(this);
    }

    public void Main(string argument, UpdateType updateSource) => navTestEnv.Tick(argument, updateSource);
}
