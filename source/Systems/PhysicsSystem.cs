using Collections.Generic;
using Physics.Events;
using Simulation;
using Simulation.Functions;
using System;
using System.Runtime.InteropServices;
using Worlds;

namespace Physics.Systems
{
    public readonly partial struct PhysicsSystem : ISystem
    {
        private readonly Dictionary<World, PhysicsSimulatorSystem> systems;

        public PhysicsSystem()
        {
            systems = new(4);
        }

        public readonly void Dispose()
        {
            foreach (PhysicsSimulatorSystem system in systems.Values)
            {
                system.Dispose();
            }

            systems.Dispose();
        }

        void ISystem.Start(in SystemContext context, in World world)
        {
            if (!systems.ContainsKey(world))
            {
                systems.Add(world, new PhysicsSimulatorSystem(world));
            }
        }

        void ISystem.Update(in SystemContext context, in World world, in TimeSpan delta)
        {
            ref PhysicsSimulatorSystem physicsSystem = ref systems.TryGetValue(world, out bool contains);
            if (!contains)
            {
                physicsSystem = new(world);
                systems.Add(world, physicsSystem);
            }

            physicsSystem.Update(delta);
        }

        void ISystem.Finish(in SystemContext context, in World world)
        {
        }

        readonly unsafe void ISystem.CollectMessageHandlers(MessageHandlerCollector collector)
        {
            collector.Add<RaycastRequest>(&HandleRaycast);
        }

        [UnmanagedCallersOnly]
        private static StatusCode HandleRaycast(HandleMessage.Input input)
        {
            ref PhysicsSystem system = ref input.ReadSystem<PhysicsSystem>();
            RaycastRequest raycast = input.ReadMessage<RaycastRequest>();
            ref PhysicsSimulatorSystem physicsSystem = ref system.systems.TryGetValue(raycast.world, out bool contains);
            if (!contains)
            {
                physicsSystem = new(raycast.world);
                system.systems.Add(raycast.world, physicsSystem);
            }

            physicsSystem.PerformRaycastRequest(raycast);
            return StatusCode.Continue;
        }
    }
}