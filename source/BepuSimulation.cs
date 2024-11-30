using BepuPhysics;
using BepuUtilities.Memory;
using System;
using System.Runtime.InteropServices;

namespace Physics.Systems
{
    public readonly struct BepuSimulation : IDisposable
    {
        private readonly GCHandle handle;

        [Obsolete("Default constructor not supported", true)]
        public BepuSimulation()
        {
            throw new NotSupportedException("Default constructor not supported");
        }

        public BepuSimulation(BufferPool bufferPool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacks poseIntegratorCallbacks, SolveDescription solveDescription)
        {
            BepuPhysics.Simulation simulation = BepuPhysics.Simulation.Create(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);
            handle = GCHandle.Alloc(simulation);
        }

        public readonly void Dispose()
        {
            BepuPhysics.Simulation simulation = this;
            simulation.Dispose();
            handle.Free();
        }

        public readonly void Update(TimeSpan delta)
        {
            BepuPhysics.Simulation simulation = this;
            simulation.Timestep((float)delta.TotalSeconds);
        }

        public static implicit operator BepuPhysics.Simulation(BepuSimulation simulation)
        {
            return (BepuPhysics.Simulation)(simulation.handle.Target ?? throw new ObjectDisposedException(nameof(BepuSimulation)));
        }
    }
}