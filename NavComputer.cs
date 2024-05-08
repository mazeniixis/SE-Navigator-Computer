 // Navigator Computer class
 public class NavComputer
 {
     readonly IMyShipController _refBlock;
     readonly List<IMyGyro> myGyros = new List<IMyGyro>();
     readonly PID _pitchPID, _yawPID;

     Vector3D _rotationPYR, _forwardVector, _upVector, _rotSpeed;
     double _slowdownAngle, _updatesPerSecond;
     public AlignMode AlignMode = AlignMode.None;
     NavStatus _navStatus = NavStatus.Off;

     public NavComputer(IMyShipController refBlock, PID pitchPID, PID yawPID, double updatesPerSecond, double slowdownAngle)
     {
         _refBlock = refBlock;
         _pitchPID = pitchPID;
         _yawPID = yawPID;
         _updatesPerSecond = updatesPerSecond;
         _slowdownAngle = slowdownAngle;
     }

     public void AddGyro(IMyGyro gyro)
     {
         if (gyro != null)
             myGyros.Add(gyro);
     }

     public void AddGyros(List<IMyGyro> gyros) => myGyros.AddRange(gyros);

     public bool AlignToHorizon()
     {
         if (Vector3D.IsZero(_upVector))
             return false;
         SetForwardVector(_upVector.Cross(_refBlock.WorldMatrix.Left));
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
             angle = Math.Acos(MathHelper.Clamp(-forwardVect.Z, -1, 1));
         }
         else
         {
             leftVector = VectorMath.SafeNormalize(leftVector);
             var upVector = Vector3D.Cross(_forwardVector, leftVector);
             var newMatrix = new MatrixD()
             {
                 Forward = forwardVect,
                 Left = leftVector,
                 Up = upVector
             };

             axis = new Vector3D(newMatrix.M32 - newMatrix.M23, newMatrix.M13 - newMatrix.M31, newMatrix.M21 - newMatrix.M12);
             double trace = newMatrix.M11 + newMatrix.M22 + newMatrix.M33;
             angle = Math.Acos(MathHelper.Clamp((trace - 1) * 5, -1, 1));

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
         if (_pitchPID != null)
             _rotSpeed.X = _pitchPID.Control(_rotationPYR.X);
         if (_yawPID != null)
             _rotSpeed.Y = _yawPID.Control(_rotationPYR.Y);

         if (Math.Abs(_rotationPYR.X) < _slowdownAngle)
             _rotSpeed.X = _updatesPerSecond * .5 * _rotationPYR.X;
         if (Math.Abs(_rotationPYR.Y) < _slowdownAngle)
             _rotSpeed.Y = _updatesPerSecond * .5 * _rotationPYR.Y;
     }

     void ApplyGyroOverride(bool bOverride = true)
     {
         var worldRotationPYR = Vector3D.TransformNormal(_rotSpeed, _refBlock.WorldMatrix);

         for (int g = 0; g < myGyros.Count; g++)
         {
             if (myGyros[g].Closed)
             {
                 myGyros.RemoveAt(g);
                 continue;
             }

             var gyro = myGyros[g];

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
         Debug("Testing...");
         Debug($"Gravity Direction: {_upVector}");
         if (_navStatus == NavStatus.Off)
             return;
         Update();
         GetRotationPYR();
         AngleController();
         ApplyGyroOverride();
     }

     public void SetStatus(NavStatus navStatus)
     {
         if (navStatus == NavStatus.Off) ApplyGyroOverride(false);
         _navStatus = navStatus;
     }
 }
