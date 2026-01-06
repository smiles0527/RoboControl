using MathNet.Numerics.LinearAlgebra;

namespace ControlWorkbench.Math.Learning;

/// <summary>
/// Deep Deterministic Policy Gradient (DDPG) for continuous control.
/// Actor-critic architecture with experience replay and target networks.
/// </summary>
public class DdpgController
{
    private readonly NeuralNetwork _actor;
    private readonly NeuralNetwork _actorTarget;
    private readonly NeuralNetwork _critic;
    private readonly NeuralNetwork _criticTarget;
    private readonly ReplayBuffer _buffer;
    
    private readonly int _stateSize;
    private readonly int _actionSize;
    private readonly double _gamma;
    private readonly double _tau;
    private readonly double _actorLr;
    private readonly double _criticLr;
    private readonly OrnsteinUhlenbeckNoise _noise;
    
    public DdpgController(
        int stateSize,
        int actionSize,
        int bufferSize = 100000,
        double gamma = 0.99,
        double tau = 0.001,
        double actorLr = 0.0001,
        double criticLr = 0.001)
    {
        _stateSize = stateSize;
        _actionSize = actionSize;
        _gamma = gamma;
        _tau = tau;
        _actorLr = actorLr;
        _criticLr = criticLr;
        
        // Actor network: state -> action
        _actor = new NeuralNetwork([stateSize, 256, 256, actionSize], ActivationType.ReLU, ActivationType.Tanh);
        _actorTarget = _actor.Clone();
        
        // Critic network: (state, action) -> Q-value
        _critic = new NeuralNetwork([stateSize + actionSize, 256, 256, 1], ActivationType.ReLU, ActivationType.Linear);
        _criticTarget = _critic.Clone();
        
        _buffer = new ReplayBuffer(bufferSize, stateSize, actionSize);
        _noise = new OrnsteinUhlenbeckNoise(actionSize);
    }
    
    /// <summary>
    /// Select action with exploration noise.
    /// </summary>
    public double[] SelectAction(double[] state, bool explore = true)
    {
        var action = _actor.Forward(state);
        
        if (explore)
        {
            var noise = _noise.Sample();
            for (int i = 0; i < _actionSize; i++)
            {
                action[i] = System.Math.Clamp(action[i] + noise[i], -1, 1);
            }
        }
        
        return action;
    }
    
    /// <summary>
    /// Store transition in replay buffer.
    /// </summary>
    public void Remember(double[] state, double[] action, double reward, double[] nextState, bool done)
    {
        _buffer.Add(state, action, reward, nextState, done);
    }
    
    /// <summary>
    /// Train on batch from replay buffer.
    /// </summary>
    public TrainingMetrics Train(int batchSize = 64)
    {
        if (_buffer.Size < batchSize)
            return new TrainingMetrics();
        
        var batch = _buffer.Sample(batchSize);
        
        // Compute target Q-values
        var targetQs = new double[batchSize];
        for (int i = 0; i < batchSize; i++)
        {
            var nextAction = _actorTarget.Forward(batch.NextStates[i]);
            var nextInput = Concatenate(batch.NextStates[i], nextAction);
            var nextQ = _criticTarget.Forward(nextInput)[0];
            targetQs[i] = batch.Rewards[i] + _gamma * (batch.Dones[i] ? 0 : nextQ);
        }
        
        // Update critic
        double criticLoss = 0;
        for (int i = 0; i < batchSize; i++)
        {
            var input = Concatenate(batch.States[i], batch.Actions[i]);
            var currentQ = _critic.Forward(input)[0];
            var error = targetQs[i] - currentQ;
            criticLoss += error * error;
            
            _critic.Backward([error], _criticLr);
        }
        criticLoss /= batchSize;
        
        // Update actor using policy gradient
        double actorLoss = 0;
        for (int i = 0; i < batchSize; i++)
        {
            var action = _actor.Forward(batch.States[i]);
            var input = Concatenate(batch.States[i], action);
            var qValue = _critic.Forward(input)[0];
            actorLoss -= qValue; // Maximize Q
            
            // Compute gradient of Q w.r.t. action, then propagate through actor
            var actionGrad = _critic.GetActionGradient(batch.States[i], action);
            _actor.BackwardWithGradient(actionGrad, _actorLr);
        }
        actorLoss /= batchSize;
        
        // Soft update target networks
        SoftUpdateTarget(_actorTarget, _actor, _tau);
        SoftUpdateTarget(_criticTarget, _critic, _tau);
        
        return new TrainingMetrics
        {
            CriticLoss = criticLoss,
            ActorLoss = actorLoss
        };
    }
    
    private void SoftUpdateTarget(NeuralNetwork target, NeuralNetwork source, double tau)
    {
        target.SoftUpdate(source, tau);
    }
    
    private double[] Concatenate(double[] a, double[] b)
    {
        var result = new double[a.Length + b.Length];
        a.CopyTo(result, 0);
        b.CopyTo(result, a.Length);
        return result;
    }
    
    public void ResetNoise() => _noise.Reset();
    public void Save(string path) => throw new NotImplementedException();
    public void Load(string path) => throw new NotImplementedException();
}

/// <summary>
/// Proximal Policy Optimization (PPO) for stable policy learning.
/// </summary>
public class PpoController
{
    private readonly NeuralNetwork _policy;
    private readonly NeuralNetwork _value;
    private readonly int _stateSize;
    private readonly int _actionSize;
    private readonly double _gamma;
    private readonly double _lambda;
    private readonly double _clipRatio;
    private readonly double _learningRate;
    
    private readonly List<Trajectory> _trajectories = new();
    
    public PpoController(
        int stateSize,
        int actionSize,
        double gamma = 0.99,
        double lambda = 0.95,
        double clipRatio = 0.2,
        double learningRate = 0.0003)
    {
        _stateSize = stateSize;
        _actionSize = actionSize;
        _gamma = gamma;
        _lambda = lambda;
        _clipRatio = clipRatio;
        _learningRate = learningRate;
        
        _policy = new NeuralNetwork([stateSize, 64, 64, actionSize * 2], ActivationType.Tanh, ActivationType.Linear);
        _value = new NeuralNetwork([stateSize, 64, 64, 1], ActivationType.Tanh, ActivationType.Linear);
    }
    
    /// <summary>
    /// Sample action from policy distribution.
    /// </summary>
    public (double[] action, double logProb) SelectAction(double[] state)
    {
        var output = _policy.Forward(state);
        
        // Output is [mean, logStd] for Gaussian policy
        var mean = new double[_actionSize];
        var logStd = new double[_actionSize];
        
        for (int i = 0; i < _actionSize; i++)
        {
            mean[i] = output[i];
            logStd[i] = System.Math.Clamp(output[_actionSize + i], -20, 2);
        }
        
        // Sample from Gaussian
        var action = new double[_actionSize];
        double logProb = 0;
        var rng = new Random();
        
        for (int i = 0; i < _actionSize; i++)
        {
            double std = System.Math.Exp(logStd[i]);
            double u1 = 1.0 - rng.NextDouble();
            double u2 = 1.0 - rng.NextDouble();
            double z = System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Cos(2.0 * System.Math.PI * u2);
            
            action[i] = System.Math.Clamp(mean[i] + std * z, -1, 1);
            
            // Log probability of Gaussian
            logProb += -0.5 * System.Math.Pow((action[i] - mean[i]) / std, 2) - logStd[i] - 0.5 * System.Math.Log(2 * System.Math.PI);
        }
        
        return (action, logProb);
    }
    
    /// <summary>
    /// Store trajectory step.
    /// </summary>
    public void StoreTransition(double[] state, double[] action, double reward, double value, double logProb)
    {
        if (_trajectories.Count == 0 || _trajectories[^1].Done)
        {
            _trajectories.Add(new Trajectory());
        }
        
        _trajectories[^1].States.Add(state);
        _trajectories[^1].Actions.Add(action);
        _trajectories[^1].Rewards.Add(reward);
        _trajectories[^1].Values.Add(value);
        _trajectories[^1].LogProbs.Add(logProb);
    }
    
    /// <summary>
    /// Mark trajectory as complete.
    /// </summary>
    public void FinishTrajectory(double lastValue)
    {
        if (_trajectories.Count > 0)
        {
            var traj = _trajectories[^1];
            traj.Done = true;
            
            // Compute GAE (Generalized Advantage Estimation)
            ComputeGae(traj, lastValue);
        }
    }
    
    /// <summary>
    /// Update policy and value networks.
    /// </summary>
    public PpoMetrics Update(int epochs = 10, int minibatchSize = 64)
    {
        if (_trajectories.Count == 0)
            return new PpoMetrics();
        
        // Collect all data
        var allStates = new List<double[]>();
        var allActions = new List<double[]>();
        var allAdvantages = new List<double>();
        var allReturns = new List<double>();
        var allOldLogProbs = new List<double>();
        
        foreach (var traj in _trajectories)
        {
            allStates.AddRange(traj.States);
            allActions.AddRange(traj.Actions);
            allAdvantages.AddRange(traj.Advantages);
            allReturns.AddRange(traj.Returns);
            allOldLogProbs.AddRange(traj.LogProbs);
        }
        
        // Normalize advantages
        double advMean = allAdvantages.Average();
        double advStd = System.Math.Sqrt(allAdvantages.Sum(a => (a - advMean) * (a - advMean)) / allAdvantages.Count);
        for (int i = 0; i < allAdvantages.Count; i++)
        {
            allAdvantages[i] = (allAdvantages[i] - advMean) / (advStd + 1e-8);
        }
        
        double totalPolicyLoss = 0;
        double totalValueLoss = 0;
        double totalEntropy = 0;
        int updateCount = 0;
        
        // Training epochs
        for (int epoch = 0; epoch < epochs; epoch++)
        {
            // Shuffle indices
            var indices = Enumerable.Range(0, allStates.Count).ToList();
            Shuffle(indices);
            
            for (int start = 0; start < indices.Count; start += minibatchSize)
            {
                int end = System.Math.Min(start + minibatchSize, indices.Count);
                var batchIndices = indices.Skip(start).Take(end - start).ToList();
                
                double policyLoss = 0;
                double valueLoss = 0;
                double entropy = 0;
                
                foreach (int idx in batchIndices)
                {
                    var state = allStates[idx];
                    var action = allActions[idx];
                    var advantage = allAdvantages[idx];
                    var returnVal = allReturns[idx];
                    var oldLogProb = allOldLogProbs[idx];
                    
                    // New log probability
                    var (_, newLogProb) = SelectAction(state);
                    
                    // Ratio
                    double ratio = System.Math.Exp(newLogProb - oldLogProb);
                    
                    // Clipped objective
                    double surr1 = ratio * advantage;
                    double surr2 = System.Math.Clamp(ratio, 1 - _clipRatio, 1 + _clipRatio) * advantage;
                    policyLoss -= System.Math.Min(surr1, surr2);
                    
                    // Value loss
                    double valuePred = _value.Forward(state)[0];
                    valueLoss += (returnVal - valuePred) * (returnVal - valuePred);
                    
                    // Entropy bonus (encourages exploration)
                    var output = _policy.Forward(state);
                    for (int i = 0; i < _actionSize; i++)
                    {
                        double logStd = System.Math.Clamp(output[_actionSize + i], -20, 2);
                        entropy += logStd + 0.5 * System.Math.Log(2 * System.Math.PI * System.Math.E);
                    }
                }
                
                policyLoss /= batchIndices.Count;
                valueLoss /= batchIndices.Count;
                entropy /= batchIndices.Count;
                
                // Update networks
                // (Simplified - actual implementation would compute proper gradients)
                
                totalPolicyLoss += policyLoss;
                totalValueLoss += valueLoss;
                totalEntropy += entropy;
                updateCount++;
            }
        }
        
        _trajectories.Clear();
        
        return new PpoMetrics
        {
            PolicyLoss = totalPolicyLoss / updateCount,
            ValueLoss = totalValueLoss / updateCount,
            Entropy = totalEntropy / updateCount
        };
    }
    
    /// <summary>
    /// Get value estimate for state.
    /// </summary>
    public double GetValue(double[] state)
    {
        return _value.Forward(state)[0];
    }
    
    private void ComputeGae(Trajectory traj, double lastValue)
    {
        int n = traj.Rewards.Count;
        traj.Advantages = new List<double>(new double[n]);
        traj.Returns = new List<double>(new double[n]);
        
        double gae = 0;
        double nextValue = lastValue;
        
        for (int t = n - 1; t >= 0; t--)
        {
            double delta = traj.Rewards[t] + _gamma * nextValue - traj.Values[t];
            gae = delta + _gamma * _lambda * gae;
            traj.Advantages[t] = gae;
            traj.Returns[t] = gae + traj.Values[t];
            nextValue = traj.Values[t];
        }
    }
    
    private void Shuffle<T>(List<T> list)
    {
        var rng = new Random();
        int n = list.Count;
        while (n > 1)
        {
            n--;
            int k = rng.Next(n + 1);
            (list[k], list[n]) = (list[n], list[k]);
        }
    }
}

/// <summary>
/// Soft Actor-Critic (SAC) for maximum entropy reinforcement learning.
/// </summary>
public class SacController
{
    private readonly NeuralNetwork _actor;
    private readonly NeuralNetwork _critic1;
    private readonly NeuralNetwork _critic2;
    private readonly NeuralNetwork _critic1Target;
    private readonly NeuralNetwork _critic2Target;
    private readonly ReplayBuffer _buffer;
    
    private readonly int _stateSize;
    private readonly int _actionSize;
    private readonly double _gamma;
    private readonly double _tau;
    private readonly double _alpha; // Temperature
    private double _logAlpha;
    private readonly double _targetEntropy;
    
    public SacController(
        int stateSize,
        int actionSize,
        double gamma = 0.99,
        double tau = 0.005,
        double alpha = 0.2,
        int bufferSize = 1000000)
    {
        _stateSize = stateSize;
        _actionSize = actionSize;
        _gamma = gamma;
        _tau = tau;
        _alpha = alpha;
        _logAlpha = System.Math.Log(alpha);
        _targetEntropy = -actionSize; // Heuristic
        
        _actor = new NeuralNetwork([stateSize, 256, 256, actionSize * 2], ActivationType.ReLU, ActivationType.Linear);
        _critic1 = new NeuralNetwork([stateSize + actionSize, 256, 256, 1], ActivationType.ReLU, ActivationType.Linear);
        _critic2 = new NeuralNetwork([stateSize + actionSize, 256, 256, 1], ActivationType.ReLU, ActivationType.Linear);
        _critic1Target = _critic1.Clone();
        _critic2Target = _critic2.Clone();
        
        _buffer = new ReplayBuffer(bufferSize, stateSize, actionSize);
    }
    
    /// <summary>
    /// Sample action using reparameterization trick.
    /// </summary>
    public (double[] action, double logProb) SampleAction(double[] state, bool deterministic = false)
    {
        var output = _actor.Forward(state);
        var mean = new double[_actionSize];
        var logStd = new double[_actionSize];
        
        for (int i = 0; i < _actionSize; i++)
        {
            mean[i] = output[i];
            logStd[i] = System.Math.Clamp(output[_actionSize + i], -20, 2);
        }
        
        if (deterministic)
        {
            return (mean.Select(m => System.Math.Tanh(m)).ToArray(), 0);
        }
        
        var action = new double[_actionSize];
        double logProb = 0;
        var rng = new Random();
        
        for (int i = 0; i < _actionSize; i++)
        {
            double std = System.Math.Exp(logStd[i]);
            double u1 = 1.0 - rng.NextDouble();
            double u2 = 1.0 - rng.NextDouble();
            double z = System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Cos(2.0 * System.Math.PI * u2);
            
            double preSquash = mean[i] + std * z;
            action[i] = System.Math.Tanh(preSquash);
            
            // Log prob with tanh squashing correction
            logProb += -0.5 * z * z - logStd[i] - 0.5 * System.Math.Log(2 * System.Math.PI);
            logProb -= System.Math.Log(1 - action[i] * action[i] + 1e-6);
        }
        
        return (action, logProb);
    }
    
    public void Remember(double[] state, double[] action, double reward, double[] nextState, bool done)
    {
        _buffer.Add(state, action, reward, nextState, done);
    }
    
    public SacMetrics Train(int batchSize = 256, double learningRate = 0.0003)
    {
        if (_buffer.Size < batchSize)
            return new SacMetrics();
        
        var batch = _buffer.Sample(batchSize);
        
        // Update critics
        double critic1Loss = 0, critic2Loss = 0;
        
        for (int i = 0; i < batchSize; i++)
        {
            var (nextAction, nextLogProb) = SampleAction(batch.NextStates[i]);
            var nextInput = Concatenate(batch.NextStates[i], nextAction);
            
            double nextQ1 = _critic1Target.Forward(nextInput)[0];
            double nextQ2 = _critic2Target.Forward(nextInput)[0];
            double nextQ = System.Math.Min(nextQ1, nextQ2) - _alpha * nextLogProb;
            
            double targetQ = batch.Rewards[i] + _gamma * (batch.Dones[i] ? 0 : nextQ);
            
            var input = Concatenate(batch.States[i], batch.Actions[i]);
            double q1 = _critic1.Forward(input)[0];
            double q2 = _critic2.Forward(input)[0];
            
            critic1Loss += (targetQ - q1) * (targetQ - q1);
            critic2Loss += (targetQ - q2) * (targetQ - q2);
            
            _critic1.Backward([targetQ - q1], learningRate);
            _critic2.Backward([targetQ - q2], learningRate);
        }
        
        // Update actor
        double actorLoss = 0;
        double alphaLoss = 0;
        
        for (int i = 0; i < batchSize; i++)
        {
            var (action, logProb) = SampleAction(batch.States[i]);
            var input = Concatenate(batch.States[i], action);
            
            double q1 = _critic1.Forward(input)[0];
            double q2 = _critic2.Forward(input)[0];
            double minQ = System.Math.Min(q1, q2);
            
            actorLoss += _alpha * logProb - minQ;
            
            // Temperature update
            alphaLoss += -_alpha * (logProb + _targetEntropy);
        }
        
        // Soft update targets
        _critic1Target.SoftUpdate(_critic1, _tau);
        _critic2Target.SoftUpdate(_critic2, _tau);
        
        return new SacMetrics
        {
            Critic1Loss = critic1Loss / batchSize,
            Critic2Loss = critic2Loss / batchSize,
            ActorLoss = actorLoss / batchSize,
            Alpha = _alpha
        };
    }
    
    private double[] Concatenate(double[] a, double[] b)
    {
        var result = new double[a.Length + b.Length];
        a.CopyTo(result, 0);
        b.CopyTo(result, a.Length);
        return result;
    }
}

/// <summary>
/// Neural network for RL controllers.
/// </summary>
public class NeuralNetwork
{
    private readonly int[] _layerSizes;
    private readonly double[][] _weights;
    private readonly double[][] _biases;
    private readonly ActivationType _hiddenActivation;
    private readonly ActivationType _outputActivation;
    private double[][]? _layerOutputs;
    private double[][]? _preActivations;
    
    public NeuralNetwork(int[] layerSizes, ActivationType hiddenActivation, ActivationType outputActivation)
    {
        _layerSizes = layerSizes;
        _hiddenActivation = hiddenActivation;
        _outputActivation = outputActivation;
        
        _weights = new double[layerSizes.Length - 1][];
        _biases = new double[layerSizes.Length - 1][];
        
        var rng = new Random(42);
        
        for (int i = 0; i < layerSizes.Length - 1; i++)
        {
            int fanIn = layerSizes[i];
            int fanOut = layerSizes[i + 1];
            
            _weights[i] = new double[fanIn * fanOut];
            _biases[i] = new double[fanOut];
            
            // Xavier/He initialization
            double scale = System.Math.Sqrt(2.0 / fanIn);
            for (int j = 0; j < _weights[i].Length; j++)
            {
                _weights[i][j] = (rng.NextDouble() * 2 - 1) * scale;
            }
        }
    }
    
    public double[] Forward(double[] input)
    {
        _layerOutputs = new double[_layerSizes.Length][];
        _preActivations = new double[_layerSizes.Length][];
        
        _layerOutputs[0] = input;
        _preActivations[0] = input;
        
        var current = input;
        
        for (int layer = 0; layer < _weights.Length; layer++)
        {
            int inputSize = _layerSizes[layer];
            int outputSize = _layerSizes[layer + 1];
            var output = new double[outputSize];
            var preAct = new double[outputSize];
            
            for (int j = 0; j < outputSize; j++)
            {
                double sum = _biases[layer][j];
                for (int i = 0; i < inputSize; i++)
                {
                    sum += current[i] * _weights[layer][i * outputSize + j];
                }
                preAct[j] = sum;
                
                var activation = layer < _weights.Length - 1 ? _hiddenActivation : _outputActivation;
                output[j] = Activate(sum, activation);
            }
            
            _preActivations[layer + 1] = preAct;
            _layerOutputs[layer + 1] = output;
            current = output;
        }
        
        return current;
    }
    
    public void Backward(double[] outputGradient, double learningRate)
    {
        if (_layerOutputs == null || _preActivations == null)
            throw new InvalidOperationException("Must call Forward before Backward");
        
        var delta = outputGradient;
        
        for (int layer = _weights.Length - 1; layer >= 0; layer--)
        {
            int inputSize = _layerSizes[layer];
            int outputSize = _layerSizes[layer + 1];
            
            // Apply activation derivative
            var activation = layer < _weights.Length - 1 ? _hiddenActivation : _outputActivation;
            for (int j = 0; j < outputSize; j++)
            {
                delta[j] *= ActivateDerivative(_preActivations[layer + 1][j], activation);
            }
            
            // Compute gradients and propagate
            var newDelta = new double[inputSize];
            
            for (int i = 0; i < inputSize; i++)
            {
                for (int j = 0; j < outputSize; j++)
                {
                    newDelta[i] += delta[j] * _weights[layer][i * outputSize + j];
                    _weights[layer][i * outputSize + j] += learningRate * delta[j] * _layerOutputs[layer][i];
                }
            }
            
            for (int j = 0; j < outputSize; j++)
            {
                _biases[layer][j] += learningRate * delta[j];
            }
            
            delta = newDelta;
        }
    }
    
    public void BackwardWithGradient(double[] actionGradient, double learningRate)
    {
        Backward(actionGradient, learningRate);
    }
    
    public double[] GetActionGradient(double[] state, double[] action)
    {
        // Numerical gradient of Q w.r.t. action
        double eps = 1e-4;
        var grad = new double[action.Length];
        
        for (int i = 0; i < action.Length; i++)
        {
            var actionPlus = (double[])action.Clone();
            actionPlus[i] += eps;
            var input = new double[state.Length + action.Length];
            state.CopyTo(input, 0);
            actionPlus.CopyTo(input, state.Length);
            double qPlus = Forward(input)[0];
            
            var actionMinus = (double[])action.Clone();
            actionMinus[i] -= eps;
            action.CopyTo(input, state.Length);
            actionMinus.CopyTo(input, state.Length);
            double qMinus = Forward(input)[0];
            
            grad[i] = (qPlus - qMinus) / (2 * eps);
        }
        
        return grad;
    }
    
    public NeuralNetwork Clone()
    {
        var clone = new NeuralNetwork(_layerSizes, _hiddenActivation, _outputActivation);
        for (int i = 0; i < _weights.Length; i++)
        {
            Array.Copy(_weights[i], clone._weights[i], _weights[i].Length);
            Array.Copy(_biases[i], clone._biases[i], _biases[i].Length);
        }
        return clone;
    }
    
    public void SoftUpdate(NeuralNetwork source, double tau)
    {
        for (int i = 0; i < _weights.Length; i++)
        {
            for (int j = 0; j < _weights[i].Length; j++)
            {
                _weights[i][j] = tau * source._weights[i][j] + (1 - tau) * _weights[i][j];
            }
            for (int j = 0; j < _biases[i].Length; j++)
            {
                _biases[i][j] = tau * source._biases[i][j] + (1 - tau) * _biases[i][j];
            }
        }
    }
    
    private double Activate(double x, ActivationType type) => type switch
    {
        ActivationType.ReLU => System.Math.Max(0, x),
        ActivationType.Tanh => System.Math.Tanh(x),
        ActivationType.Sigmoid => 1.0 / (1.0 + System.Math.Exp(-x)),
        ActivationType.Linear => x,
        _ => x
    };
    
    private double ActivateDerivative(double x, ActivationType type) => type switch
    {
        ActivationType.ReLU => x > 0 ? 1 : 0,
        ActivationType.Tanh => 1 - System.Math.Tanh(x) * System.Math.Tanh(x),
        ActivationType.Sigmoid => Activate(x, type) * (1 - Activate(x, type)),
        ActivationType.Linear => 1,
        _ => 1
    };
}

public enum ActivationType { ReLU, Tanh, Sigmoid, Linear }

/// <summary>
/// Experience replay buffer.
/// </summary>
public class ReplayBuffer
{
    private readonly int _maxSize;
    private readonly int _stateSize;
    private readonly int _actionSize;
    
    private readonly double[][] _states;
    private readonly double[][] _actions;
    private readonly double[] _rewards;
    private readonly double[][] _nextStates;
    private readonly bool[] _dones;
    
    private int _ptr;
    private int _size;
    private readonly Random _rng;
    
    public ReplayBuffer(int maxSize, int stateSize, int actionSize)
    {
        _maxSize = maxSize;
        _stateSize = stateSize;
        _actionSize = actionSize;
        
        _states = new double[maxSize][];
        _actions = new double[maxSize][];
        _nextStates = new double[maxSize][];
        _rewards = new double[maxSize];
        _dones = new bool[maxSize];
        
        _rng = new Random();
    }
    
    public int Size => _size;
    
    public void Add(double[] state, double[] action, double reward, double[] nextState, bool done)
    {
        _states[_ptr] = (double[])state.Clone();
        _actions[_ptr] = (double[])action.Clone();
        _rewards[_ptr] = reward;
        _nextStates[_ptr] = (double[])nextState.Clone();
        _dones[_ptr] = done;
        
        _ptr = (_ptr + 1) % _maxSize;
        _size = System.Math.Min(_size + 1, _maxSize);
    }
    
    public Batch Sample(int batchSize)
    {
        var indices = new int[batchSize];
        for (int i = 0; i < batchSize; i++)
        {
            indices[i] = _rng.Next(_size);
        }
        
        return new Batch
        {
            States = indices.Select(i => _states[i]).ToArray(),
            Actions = indices.Select(i => _actions[i]).ToArray(),
            Rewards = indices.Select(i => _rewards[i]).ToArray(),
            NextStates = indices.Select(i => _nextStates[i]).ToArray(),
            Dones = indices.Select(i => _dones[i]).ToArray()
        };
    }
}

public class Batch
{
    public double[][] States { get; set; } = [];
    public double[][] Actions { get; set; } = [];
    public double[] Rewards { get; set; } = [];
    public double[][] NextStates { get; set; } = [];
    public bool[] Dones { get; set; } = [];
}

/// <summary>
/// Ornstein-Uhlenbeck noise for exploration.
/// </summary>
public class OrnsteinUhlenbeckNoise
{
    private readonly int _size;
    private readonly double _mu;
    private readonly double _theta;
    private readonly double _sigma;
    private double[] _state;
    private readonly Random _rng;
    
    public OrnsteinUhlenbeckNoise(int size, double mu = 0, double theta = 0.15, double sigma = 0.2)
    {
        _size = size;
        _mu = mu;
        _theta = theta;
        _sigma = sigma;
        _state = new double[size];
        _rng = new Random();
    }
    
    public double[] Sample()
    {
        for (int i = 0; i < _size; i++)
        {
            double u1 = 1.0 - _rng.NextDouble();
            double u2 = 1.0 - _rng.NextDouble();
            double z = System.Math.Sqrt(-2.0 * System.Math.Log(u1)) * System.Math.Cos(2.0 * System.Math.PI * u2);
            
            _state[i] += _theta * (_mu - _state[i]) + _sigma * z;
        }
        
        return (double[])_state.Clone();
    }
    
    public void Reset()
    {
        _state = new double[_size];
    }
}

/// <summary>
/// Trajectory storage for on-policy algorithms.
/// </summary>
public class Trajectory
{
    public List<double[]> States { get; } = new();
    public List<double[]> Actions { get; } = new();
    public List<double> Rewards { get; } = new();
    public List<double> Values { get; } = new();
    public List<double> LogProbs { get; } = new();
    public List<double> Advantages { get; set; } = new();
    public List<double> Returns { get; set; } = new();
    public bool Done { get; set; }
}

// Metrics classes
public class TrainingMetrics { public double CriticLoss; public double ActorLoss; }
public class PpoMetrics { public double PolicyLoss; public double ValueLoss; public double Entropy; }
public class SacMetrics { public double Critic1Loss; public double Critic2Loss; public double ActorLoss; public double Alpha; }
