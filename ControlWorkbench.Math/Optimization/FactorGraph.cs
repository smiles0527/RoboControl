using MathNet.Numerics.LinearAlgebra;
using ControlWorkbench.Math.Geometry;

namespace ControlWorkbench.Math.Optimization;

/// <summary>
/// Factor Graph implementation for nonlinear least-squares optimization.
/// Supports incremental optimization (iSAM2-style) with Bayes tree factorization.
/// Used for SLAM, sensor fusion, and multi-robot state estimation.
/// 
/// Based on:
/// - "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (Kaess et al., 2012)
/// - "Factor Graphs for Robot Perception" (Dellaert & Kaess, 2017)
/// </summary>
public class FactorGraph
{
    private readonly List<Variable> _variables = new();
    private readonly List<Factor> _factors = new();
    private readonly Dictionary<string, int> _variableIndex = new();
    
    private BayesTree? _bayesTree;
    private Matrix<double>? _linearSystem;
    private Vector<double>? _rhs;
    private bool _needsRelinearization;
    
    private readonly FactorGraphConfig _config;
    
    public int VariableCount => _variables.Count;
    public int FactorCount => _factors.Count;
    
    public FactorGraph(FactorGraphConfig? config = null)
    {
        _config = config ?? new FactorGraphConfig();
    }
    
    #region Variable Management
    
    /// <summary>
    /// Add a pose variable (SE(3)) to the graph.
    /// </summary>
    public void AddPoseVariable(string key, Matrix<double> initialValue)
    {
        var variable = new PoseVariable(key, initialValue);
        _variables.Add(variable);
        _variableIndex[key] = _variables.Count - 1;
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a landmark/point variable (R^3) to the graph.
    /// </summary>
    public void AddPointVariable(string key, Vector<double> initialValue)
    {
        var variable = new PointVariable(key, initialValue);
        _variables.Add(variable);
        _variableIndex[key] = _variables.Count - 1;
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a velocity variable (R^3) to the graph.
    /// </summary>
    public void AddVelocityVariable(string key, Vector<double> initialValue)
    {
        var variable = new VelocityVariable(key, initialValue);
        _variables.Add(variable);
        _variableIndex[key] = _variables.Count - 1;
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add an IMU bias variable to the graph.
    /// </summary>
    public void AddBiasVariable(string key, Vector<double> gyro, Vector<double> accel)
    {
        var bias = Vector<double>.Build.Dense(6);
        bias.SetSubVector(0, 3, gyro);
        bias.SetSubVector(3, 3, accel);
        var variable = new BiasVariable(key, bias);
        _variables.Add(variable);
        _variableIndex[key] = _variables.Count - 1;
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Get variable value by key.
    /// </summary>
    public T GetVariable<T>(string key) where T : Variable
    {
        if (!_variableIndex.TryGetValue(key, out int index))
            throw new KeyNotFoundException($"Variable {key} not found");
        return (T)_variables[index];
    }
    
    /// <summary>
    /// Check if variable exists.
    /// </summary>
    public bool HasVariable(string key) => _variableIndex.ContainsKey(key);
    
    #endregion
    
    #region Factor Management
    
    /// <summary>
    /// Add a prior factor on a pose.
    /// </summary>
    public void AddPosePrior(string key, Matrix<double> prior, Matrix<double> covariance)
    {
        var factor = new PosePriorFactor(key, prior, covariance);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a between factor (odometry) between two poses.
    /// </summary>
    public void AddBetweenFactor(string key1, string key2, Matrix<double> relativePose, Matrix<double> covariance)
    {
        var factor = new BetweenFactor(key1, key2, relativePose, covariance);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a landmark observation factor.
    /// </summary>
    public void AddProjectionFactor(string poseKey, string landmarkKey, 
        Vector<double> observation, Matrix<double> covariance, CameraCalibration calibration)
    {
        var factor = new ProjectionFactor(poseKey, landmarkKey, observation, covariance, calibration);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add an IMU factor between poses.
    /// </summary>
    public void AddImuFactor(string pose1Key, string vel1Key, string pose2Key, string vel2Key,
        string biasKey, ImuPreintegrationMeasurement preint)
    {
        var factor = new ImuFactor(pose1Key, vel1Key, pose2Key, vel2Key, biasKey, preint);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a GPS factor to a pose.
    /// </summary>
    public void AddGpsFactor(string poseKey, Vector<double> gpsPosition, Matrix<double> covariance)
    {
        var factor = new GpsFactor(poseKey, gpsPosition, covariance);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a range factor between two variables.
    /// </summary>
    public void AddRangeFactor(string key1, string key2, double range, double sigma)
    {
        var factor = new RangeFactor(key1, key2, range, sigma);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    /// <summary>
    /// Add a loop closure factor.
    /// </summary>
    public void AddLoopClosureFactor(string key1, string key2, Matrix<double> relativePose, 
        Matrix<double> covariance, double robustThreshold = 3.0)
    {
        var factor = new RobustBetweenFactor(key1, key2, relativePose, covariance, robustThreshold);
        _factors.Add(factor);
        _needsRelinearization = true;
    }
    
    #endregion
    
    #region Optimization
    
    /// <summary>
    /// Perform batch optimization using Levenberg-Marquardt.
    /// </summary>
    public OptimizationResult Optimize(int maxIterations = 100)
    {
        var result = new OptimizationResult();
        double prevError = double.MaxValue;
        double lambda = _config.InitialLambda;
        
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // Linearize
            var (H, b, error) = Linearize();
            
            result.Iterations = iter + 1;
            result.FinalError = error;
            
            // Check convergence
            if (System.Math.Abs(prevError - error) < _config.RelativeErrorTolerance * prevError)
            {
                result.Converged = true;
                break;
            }
            
            // LM damping
            var Hdamped = H.Clone();
            for (int i = 0; i < H.RowCount; i++)
                Hdamped[i, i] += lambda * H[i, i];
            
            // Solve
            Vector<double> delta;
            try
            {
                delta = Hdamped.Solve(b);
            }
            catch
            {
                // Increase damping and retry
                lambda *= 10;
                continue;
            }
            
            // Compute error with update
            ApplyUpdate(delta);
            var (_, _, newError) = Linearize();
            
            if (newError < error)
            {
                // Accept update
                prevError = error;
                lambda = System.Math.Max(lambda / _config.LambdaDecreaseFactor, 1e-7);
            }
            else
            {
                // Reject update, revert
                ApplyUpdate(-delta);
                lambda = System.Math.Min(lambda * _config.LambdaIncreaseFactor, 1e7);
            }
            
            if (delta.L2Norm() < _config.AbsoluteErrorTolerance)
            {
                result.Converged = true;
                break;
            }
        }
        
        return result;
    }
    
    /// <summary>
    /// Perform incremental optimization (iSAM2-style).
    /// Only relinearizes and updates affected variables.
    /// </summary>
    public OptimizationResult OptimizeIncremental()
    {
        if (_bayesTree == null || _needsRelinearization)
        {
            // Full relinearization
            _bayesTree = BuildBayesTree();
            _needsRelinearization = false;
        }
        else
        {
            // Incremental update
            UpdateBayesTree();
        }
        
        // Solve using Bayes tree
        var delta = _bayesTree.Solve();
        ApplyUpdate(delta);
        
        var (_, _, error) = Linearize();
        
        return new OptimizationResult
        {
            Converged = true,
            FinalError = error,
            Iterations = 1
        };
    }
    
    /// <summary>
    /// Get marginal covariance for a variable.
    /// </summary>
    public Matrix<double> GetMarginalCovariance(string key)
    {
        if (_bayesTree == null)
        {
            var (H, _, _) = Linearize();
            var HInv = H.PseudoInverse();
            
            int index = _variableIndex[key];
            int start = GetVariableStartIndex(index);
            int dim = _variables[index].Dimension;
            
            return HInv.SubMatrix(start, dim, start, dim);
        }
        
        return _bayesTree.GetMarginal(key);
    }
    
    #endregion
    
    #region Internal Methods
    
    private (Matrix<double> H, Vector<double> b, double error) Linearize()
    {
        int totalDim = _variables.Sum(v => v.Dimension);
        var H = Matrix<double>.Build.Dense(totalDim, totalDim);
        var b = Vector<double>.Build.Dense(totalDim);
        double totalError = 0;
        
        foreach (var factor in _factors)
        {
            var (Js, r, W) = factor.Linearize(_variables, _variableIndex);
            
            // Compute weighted error
            var Wr = W * r;
            totalError += 0.5 * (r * Wr);
            
            // Accumulate H and b
            for (int i = 0; i < factor.Keys.Length; i++)
            {
                int idx_i = _variableIndex[factor.Keys[i]];
                int start_i = GetVariableStartIndex(idx_i);
                int dim_i = _variables[idx_i].Dimension;
                
                var Ji = Js[i];
                var WJi = W * Ji;
                
                // b += J^T W r
                var JiTWr = Ji.Transpose() * Wr;
                for (int k = 0; k < dim_i; k++)
                    b[start_i + k] += JiTWr[k];
                
                for (int j = 0; j < factor.Keys.Length; j++)
                {
                    int idx_j = _variableIndex[factor.Keys[j]];
                    int start_j = GetVariableStartIndex(idx_j);
                    int dim_j = _variables[idx_j].Dimension;
                    
                    var Jj = Js[j];
                    
                    // H += J_i^T W J_j
                    var JiTWJj = Ji.Transpose() * W * Jj;
                    for (int ki = 0; ki < dim_i; ki++)
                    {
                        for (int kj = 0; kj < dim_j; kj++)
                        {
                            H[start_i + ki, start_j + kj] += JiTWJj[ki, kj];
                        }
                    }
                }
            }
        }
        
        return (H, b, totalError);
    }
    
    private void ApplyUpdate(Vector<double> delta)
    {
        int offset = 0;
        foreach (var variable in _variables)
        {
            var dv = delta.SubVector(offset, variable.Dimension);
            variable.ApplyUpdate(dv);
            offset += variable.Dimension;
        }
    }
    
    private int GetVariableStartIndex(int variableIndex)
    {
        int start = 0;
        for (int i = 0; i < variableIndex; i++)
            start += _variables[i].Dimension;
        return start;
    }
    
    private BayesTree BuildBayesTree()
    {
        // Compute elimination ordering using COLAMD or nested dissection
        var ordering = ComputeOrdering();
        
        // Eliminate variables to form Bayes tree
        var tree = new BayesTree(_variables, _variableIndex, ordering);
        
        // Add factors to tree
        foreach (var factor in _factors)
        {
            tree.AddFactor(factor);
        }
        
        tree.Eliminate();
        
        return tree;
    }
    
    private void UpdateBayesTree()
    {
        // Find affected cliques and relinearize only those
        // This is the key insight of iSAM2
        _bayesTree!.IncrementalUpdate(_factors.TakeLast(1).ToList());
    }
    
    private int[] ComputeOrdering()
    {
        // Simple reverse ordering (COLAMD would be better)
        return Enumerable.Range(0, _variables.Count).Reverse().ToArray();
    }
    
    #endregion
}

#region Variable Types

public abstract class Variable
{
    public string Key { get; }
    public abstract int Dimension { get; }
    
    protected Variable(string key) => Key = key;
    
    public abstract void ApplyUpdate(Vector<double> delta);
    public abstract Variable Clone();
}

public class PoseVariable : Variable
{
    public Matrix<double> Value { get; private set; }
    public override int Dimension => 6; // SE(3) has 6 DOF
    
    public PoseVariable(string key, Matrix<double> value) : base(key)
    {
        Value = value.Clone();
    }
    
    public override void ApplyUpdate(Vector<double> delta)
    {
        Value = LieGroups.BoxPlusSE3(Value, delta);
    }
    
    public override Variable Clone() => new PoseVariable(Key, Value);
}

public class PointVariable : Variable
{
    public Vector<double> Value { get; private set; }
    public override int Dimension => 3;
    
    public PointVariable(string key, Vector<double> value) : base(key)
    {
        Value = value.Clone();
    }
    
    public override void ApplyUpdate(Vector<double> delta)
    {
        Value += delta;
    }
    
    public override Variable Clone() => new PointVariable(Key, Value);
}

public class VelocityVariable : Variable
{
    public Vector<double> Value { get; private set; }
    public override int Dimension => 3;
    
    public VelocityVariable(string key, Vector<double> value) : base(key)
    {
        Value = value.Clone();
    }
    
    public override void ApplyUpdate(Vector<double> delta)
    {
        Value += delta;
    }
    
    public override Variable Clone() => new VelocityVariable(Key, Value);
}

public class BiasVariable : Variable
{
    public Vector<double> Value { get; private set; } // [gyro; accel]
    public override int Dimension => 6;
    
    public BiasVariable(string key, Vector<double> value) : base(key)
    {
        Value = value.Clone();
    }
    
    public override void ApplyUpdate(Vector<double> delta)
    {
        Value += delta;
    }
    
    public Vector<double> GyroBias => Value.SubVector(0, 3);
    public Vector<double> AccelBias => Value.SubVector(3, 3);
    
    public override Variable Clone() => new BiasVariable(Key, Value);
}

#endregion

#region Factor Types

public abstract class Factor
{
    public abstract string[] Keys { get; }
    
    public abstract (Matrix<double>[] Jacobians, Vector<double> residual, Matrix<double> information) 
        Linearize(List<Variable> variables, Dictionary<string, int> variableIndex);
}

public class PosePriorFactor : Factor
{
    public override string[] Keys { get; }
    private readonly Matrix<double> _prior;
    private readonly Matrix<double> _information;
    
    public PosePriorFactor(string key, Matrix<double> prior, Matrix<double> covariance)
    {
        Keys = [key];
        _prior = prior;
        _information = covariance.PseudoInverse();
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var pose = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        
        // Residual in tangent space
        var residual = LieGroups.BoxMinusSE3(pose, _prior);
        
        // Jacobian is identity for prior factor
        var J = Matrix<double>.Build.DenseIdentity(6);
        
        return ([J], residual, _information);
    }
}

public class BetweenFactor : Factor
{
    public override string[] Keys { get; }
    private readonly Matrix<double> _measurement;
    private readonly Matrix<double> _information;
    
    public BetweenFactor(string key1, string key2, Matrix<double> measurement, Matrix<double> covariance)
    {
        Keys = [key1, key2];
        _measurement = measurement;
        _information = covariance.PseudoInverse();
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var T1 = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        var T2 = ((PoseVariable)variables[variableIndex[Keys[1]]]).Value;
        
        // Predicted relative pose
        var T12_pred = LieGroups.RelativeSE3(T1, T2);
        
        // Residual
        var residual = LieGroups.BoxMinusSE3(T12_pred, _measurement);
        
        // Jacobians using adjoint
        var J1 = -LieGroups.AdjointSE3(LieGroups.InverseSE3(T12_pred));
        var J2 = Matrix<double>.Build.DenseIdentity(6);
        
        return ([J1, J2], residual, _information);
    }
}

public class RobustBetweenFactor : Factor
{
    public override string[] Keys { get; }
    private readonly Matrix<double> _measurement;
    private readonly Matrix<double> _information;
    private readonly double _threshold;
    
    public RobustBetweenFactor(string key1, string key2, Matrix<double> measurement, 
        Matrix<double> covariance, double threshold)
    {
        Keys = [key1, key2];
        _measurement = measurement;
        _information = covariance.PseudoInverse();
        _threshold = threshold;
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var T1 = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        var T2 = ((PoseVariable)variables[variableIndex[Keys[1]]]).Value;
        
        var T12_pred = LieGroups.RelativeSE3(T1, T2);
        var residual = LieGroups.BoxMinusSE3(T12_pred, _measurement);
        
        // Compute Mahalanobis distance for robust kernel
        double mahaDist = System.Math.Sqrt(residual * _information * residual);
        
        // Huber robust kernel
        double weight = mahaDist <= _threshold ? 1.0 : _threshold / mahaDist;
        var robustInfo = _information * weight;
        
        var J1 = -LieGroups.AdjointSE3(LieGroups.InverseSE3(T12_pred));
        var J2 = Matrix<double>.Build.DenseIdentity(6);
        
        return ([J1, J2], residual, robustInfo);
    }
}

public class ProjectionFactor : Factor
{
    public override string[] Keys { get; }
    private readonly Vector<double> _observation;
    private readonly Matrix<double> _information;
    private readonly CameraCalibration _calibration;
    
    public ProjectionFactor(string poseKey, string landmarkKey, 
        Vector<double> observation, Matrix<double> covariance, CameraCalibration calibration)
    {
        Keys = [poseKey, landmarkKey];
        _observation = observation;
        _information = covariance.PseudoInverse();
        _calibration = calibration;
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var pose = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        var landmark = ((PointVariable)variables[variableIndex[Keys[1]]]).Value;
        
        // Transform point to camera frame
        var R = pose.SubMatrix(0, 3, 0, 3);
        var t = Vector<double>.Build.DenseOfArray([pose[0, 3], pose[1, 3], pose[2, 3]]);
        var pointCam = R.Transpose() * (landmark - t);
        
        // Project to image
        double x = pointCam[0], y = pointCam[1], z = pointCam[2];
        var projected = Vector<double>.Build.DenseOfArray([
            _calibration.Fx * x / z + _calibration.Cx,
            _calibration.Fy * y / z + _calibration.Cy
        ]);
        
        var residual = projected - _observation;
        
        // Jacobian w.r.t. point in camera frame
        var Jproj = Matrix<double>.Build.DenseOfArray(new double[,]
        {
            { _calibration.Fx / z, 0, -_calibration.Fx * x / (z * z) },
            { 0, _calibration.Fy / z, -_calibration.Fy * y / (z * z) }
        });
        
        // Jacobian w.r.t. pose (using SE(3) perturbation)
        var pointSkew = LieGroups.Skew3(pointCam);
        var JposeCam = Matrix<double>.Build.Dense(3, 6);
        JposeCam.SetSubMatrix(0, 0, pointSkew);
        JposeCam.SetSubMatrix(0, 3, -Matrix<double>.Build.DenseIdentity(3));
        var Jpose = Jproj * JposeCam;
        
        // Jacobian w.r.t. landmark
        var Jlandmark = Jproj * R.Transpose();
        
        return ([Jpose, Jlandmark], residual, _information);
    }
}

public class ImuFactor : Factor
{
    public override string[] Keys { get; }
    private readonly ImuPreintegrationMeasurement _preint;
    
    public ImuFactor(string pose1Key, string vel1Key, string pose2Key, string vel2Key,
        string biasKey, ImuPreintegrationMeasurement preint)
    {
        Keys = [pose1Key, vel1Key, pose2Key, vel2Key, biasKey];
        _preint = preint;
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var T1 = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        var v1 = ((VelocityVariable)variables[variableIndex[Keys[1]]]).Value;
        var T2 = ((PoseVariable)variables[variableIndex[Keys[2]]]).Value;
        var v2 = ((VelocityVariable)variables[variableIndex[Keys[3]]]).Value;
        var bias = ((BiasVariable)variables[variableIndex[Keys[4]]]).Value;
        
        var R1 = T1.SubMatrix(0, 3, 0, 3);
        var t1 = Vector<double>.Build.DenseOfArray([T1[0, 3], T1[1, 3], T1[2, 3]]);
        var R2 = T2.SubMatrix(0, 3, 0, 3);
        var t2 = Vector<double>.Build.DenseOfArray([T2[0, 3], T2[1, 3], T2[2, 3]]);
        
        double dt = _preint.DeltaT;
        var gravity = Vector<double>.Build.DenseOfArray([0, 0, -9.81]);
        
        // Predicted values from preintegration
        var R_pred = R1 * _preint.DeltaR;
        var v_pred = v1 + gravity * dt + R1 * _preint.DeltaV;
        var t_pred = t1 + v1 * dt + 0.5 * gravity * dt * dt + R1 * _preint.DeltaP;
        
        // Residuals (15-dimensional: 3 rotation, 3 velocity, 3 position, 3 gyro bias, 3 accel bias)
        var residual = Vector<double>.Build.Dense(15);
        residual.SetSubVector(0, 3, LieGroups.LogSO3(R_pred.Transpose() * R2));
        residual.SetSubVector(3, 3, v2 - v_pred);
        residual.SetSubVector(6, 3, t2 - t_pred);
        residual.SetSubVector(9, 6, bias - _preint.Bias);
        
        // Jacobians (simplified - full implementation would include bias correction)
        var J1 = Matrix<double>.Build.Dense(15, 6);
        var Jv1 = Matrix<double>.Build.Dense(15, 3);
        var J2 = Matrix<double>.Build.Dense(15, 6);
        var Jv2 = Matrix<double>.Build.Dense(15, 3);
        var Jbias = Matrix<double>.Build.Dense(15, 6);
        
        // Fill in key Jacobian blocks
        J1.SetSubMatrix(0, 0, -_preint.DeltaR.Transpose() * R1.Transpose());
        J1.SetSubMatrix(6, 3, -Matrix<double>.Build.DenseIdentity(3));
        
        Jv1.SetSubMatrix(3, 0, -Matrix<double>.Build.DenseIdentity(3));
        Jv1.SetSubMatrix(6, 0, -Matrix<double>.Build.DenseIdentity(3) * dt);
        
        J2.SetSubMatrix(0, 0, Matrix<double>.Build.DenseIdentity(3));
        J2.SetSubMatrix(6, 3, Matrix<double>.Build.DenseIdentity(3));
        
        Jv2.SetSubMatrix(3, 0, Matrix<double>.Build.DenseIdentity(3));
        
        Jbias.SetSubMatrix(9, 0, Matrix<double>.Build.DenseIdentity(6));
        
        return ([J1, Jv1, J2, Jv2, Jbias], residual, _preint.Covariance.PseudoInverse());
    }
}

public class GpsFactor : Factor
{
    public override string[] Keys { get; }
    private readonly Vector<double> _position;
    private readonly Matrix<double> _information;
    
    public GpsFactor(string poseKey, Vector<double> position, Matrix<double> covariance)
    {
        Keys = [poseKey];
        _position = position;
        _information = covariance.PseudoInverse();
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var pose = ((PoseVariable)variables[variableIndex[Keys[0]]]).Value;
        var position = Vector<double>.Build.DenseOfArray([pose[0, 3], pose[1, 3], pose[2, 3]]);
        
        var residual = position - _position;
        
        // Jacobian: only position part of pose matters
        var J = Matrix<double>.Build.Dense(3, 6);
        J.SetSubMatrix(0, 3, Matrix<double>.Build.DenseIdentity(3));
        
        return ([J], residual, _information);
    }
}

public class RangeFactor : Factor
{
    public override string[] Keys { get; }
    private readonly double _range;
    private readonly double _sigma;
    
    public RangeFactor(string key1, string key2, double range, double sigma)
    {
        Keys = [key1, key2];
        _range = range;
        _sigma = sigma;
    }
    
    public override (Matrix<double>[], Vector<double>, Matrix<double>) Linearize(
        List<Variable> variables, Dictionary<string, int> variableIndex)
    {
        var var1 = variables[variableIndex[Keys[0]]];
        var var2 = variables[variableIndex[Keys[1]]];
        
        Vector<double> p1, p2;
        
        if (var1 is PoseVariable pv1)
            p1 = Vector<double>.Build.DenseOfArray([pv1.Value[0, 3], pv1.Value[1, 3], pv1.Value[2, 3]]);
        else
            p1 = ((PointVariable)var1).Value;
        
        if (var2 is PoseVariable pv2)
            p2 = Vector<double>.Build.DenseOfArray([pv2.Value[0, 3], pv2.Value[1, 3], pv2.Value[2, 3]]);
        else
            p2 = ((PointVariable)var2).Value;
        
        var diff = p1 - p2;
        double dist = diff.L2Norm();
        var residual = Vector<double>.Build.DenseOfArray([dist - _range]);
        
        var direction = diff / dist;
        
        var J1 = var1 is PoseVariable 
            ? Matrix<double>.Build.Dense(1, 6, (i, j) => j >= 3 ? direction[j - 3] : 0)
            : Matrix<double>.Build.DenseOfRowVectors(direction);
        
        var J2 = var2 is PoseVariable 
            ? Matrix<double>.Build.Dense(1, 6, (i, j) => j >= 3 ? -direction[j - 3] : 0)
            : Matrix<double>.Build.DenseOfRowVectors(-direction);
        
        var information = Matrix<double>.Build.Dense(1, 1, 1.0 / (_sigma * _sigma));
        
        return ([J1, J2], residual, information);
    }
}

#endregion

#region Supporting Types

public class CameraCalibration
{
    public double Fx { get; set; }
    public double Fy { get; set; }
    public double Cx { get; set; }
    public double Cy { get; set; }
    public double[] DistortionCoeffs { get; set; } = [];
}

public class ImuPreintegrationMeasurement
{
    public Matrix<double> DeltaR { get; set; } = Matrix<double>.Build.DenseIdentity(3);
    public Vector<double> DeltaV { get; set; } = Vector<double>.Build.Dense(3);
    public Vector<double> DeltaP { get; set; } = Vector<double>.Build.Dense(3);
    public double DeltaT { get; set; }
    public Matrix<double> Covariance { get; set; } = Matrix<double>.Build.DenseIdentity(15);
    public Vector<double> Bias { get; set; } = Vector<double>.Build.Dense(6);
}

public class BayesTree
{
    private readonly List<Variable> _variables;
    private readonly Dictionary<string, int> _variableIndex;
    private readonly int[] _ordering;
    private readonly List<Factor> _factors = new();
    
    private Matrix<double>? _R;
    private Vector<double>? _d;
    
    public BayesTree(List<Variable> variables, Dictionary<string, int> variableIndex, int[] ordering)
    {
        _variables = variables;
        _variableIndex = variableIndex;
        _ordering = ordering;
    }
    
    public void AddFactor(Factor factor)
    {
        _factors.Add(factor);
    }
    
    public void Eliminate()
    {
        // Build full Jacobian and eliminate
        int totalDim = _variables.Sum(v => v.Dimension);
        _R = Matrix<double>.Build.Dense(totalDim, totalDim);
        _d = Vector<double>.Build.Dense(totalDim);
        
        // QR factorization following elimination ordering
        // (Simplified - full implementation would build proper clique tree)
        foreach (var factor in _factors)
        {
            var (Js, r, W) = factor.Linearize(_variables, _variableIndex);
            // Accumulate into R and d
        }
    }
    
    public void IncrementalUpdate(List<Factor> newFactors)
    {
        // Add new factors and update affected cliques
        foreach (var factor in newFactors)
        {
            _factors.Add(factor);
        }
        
        // Re-eliminate affected variables
        Eliminate();
    }
    
    public Vector<double> Solve()
    {
        if (_R == null || _d == null)
            throw new InvalidOperationException("Tree not eliminated");
        
        // Back-substitution
        int n = _d.Count;
        var x = Vector<double>.Build.Dense(n);
        
        for (int i = n - 1; i >= 0; i--)
        {
            double sum = _d[i];
            for (int j = i + 1; j < n; j++)
                sum -= _R[i, j] * x[j];
            x[i] = sum / (_R[i, i] + 1e-10);
        }
        
        return x;
    }
    
    public Matrix<double> GetMarginal(string key)
    {
        if (_R == null)
            throw new InvalidOperationException("Tree not eliminated");
        
        int idx = _variableIndex[key];
        int start = 0;
        for (int i = 0; i < idx; i++)
            start += _variables[i].Dimension;
        int dim = _variables[idx].Dimension;
        
        var RInv = _R.PseudoInverse();
        return RInv.SubMatrix(start, dim, start, dim);
    }
}

public class FactorGraphConfig
{
    public double InitialLambda { get; set; } = 1e-5;
    public double LambdaIncreaseFactor { get; set; } = 10;
    public double LambdaDecreaseFactor { get; set; } = 3;
    public double RelativeErrorTolerance { get; set; } = 1e-5;
    public double AbsoluteErrorTolerance { get; set; } = 1e-5;
}

public class OptimizationResult
{
    public bool Converged { get; set; }
    public double FinalError { get; set; }
    public int Iterations { get; set; }
}

#endregion
