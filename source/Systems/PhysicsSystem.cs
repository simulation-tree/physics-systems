using Collections;
using Physics.Events;
using Simulation;
using System;
using System.Runtime.InteropServices;
using Unmanaged;
using Worlds;

namespace Physics.Systems
{
    public readonly partial struct PhysicsSystem : ISystem
    {
        private readonly Dictionary<World, PhysicsSimulatorSystem> systems;

        void ISystem.Start(in SystemContainer systemContainer, in World world)
        {
            if (!systems.ContainsKey(world))
            {
                systems.Add(world, new PhysicsSimulatorSystem(world));
            }
        }

        void ISystem.Update(in SystemContainer systemContainer, in World world, in TimeSpan delta)
        {
            ref PhysicsSimulatorSystem physicsSystem = ref systems.TryGetValue(world, out bool contains);
            if (!contains)
            {
                physicsSystem = new(world);
                systems.Add(world, physicsSystem);
            }

            physicsSystem.Update(delta);
        }

        void ISystem.Finish(in SystemContainer systemContainer, in World world)
        {
        }

        readonly unsafe uint ISystem.GetMessageHandlers(USpan<MessageHandler> buffer)
        {
            buffer[0] = MessageHandler.Create<RaycastRequest>(new(&HandleRaycast));
            return 1;
        }

        [UnmanagedCallersOnly]
        private static void HandleRaycast(SystemContainer container, World world, Allocation message)
        {
            ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
            RaycastRequest raycast = message.Read<RaycastRequest>();
            ref PhysicsSimulatorSystem physicsSystem = ref system.systems.TryGetValue(world, out bool contains);
            if (!contains)
            {
                physicsSystem = new(world);
                system.systems.Add(world, physicsSystem);
            }

            physicsSystem.PerformRaycastRequest(raycast);
        }

        public PhysicsSystem()
        {
            systems = new();
        }

        public readonly void Dispose()
        {
            foreach (World world in systems.Keys)
            {
                systems[world].Dispose();
            }

            systems.Dispose();
        }
    }
}