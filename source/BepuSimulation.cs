using BepuPhysics;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
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

        public readonly void RemoveDynamicBody(BodyHandle dynamicBody)
        {
            BepuPhysics.Simulation simulation = this;
            simulation.Bodies.Remove(dynamicBody);
        }

        public readonly void RemoveStaticBody(StaticHandle staticBody)
        {
            BepuPhysics.Simulation simulation = this;
            simulation.Statics.Remove(staticBody);
        }

        public readonly StaticReference GetStaticBody(StaticHandle staticBody)
        {
            BepuPhysics.Simulation simulation = this;
            return simulation.Statics.GetStaticReference(staticBody);
        }

        public readonly BodyReference GetDynamicBody(BodyHandle dynamicBody)
        {
            BepuPhysics.Simulation simulation = this;
            return simulation.Bodies.GetBodyReference(dynamicBody);
        }

        public static implicit operator BepuPhysics.Simulation(BepuSimulation simulation)
        {
            ThrowIfDisposed(simulation);

            return (BepuPhysics.Simulation)simulation.handle.Target!;
        }

        [Conditional("DEBUG")]
        private static void ThrowIfDisposed(BepuSimulation simulation)
        {
            if (simulation.handle.Target is null)
            {
                throw new ObjectDisposedException(nameof(BepuSimulation));
            }
        }
    }
}