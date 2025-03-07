using Collections.Generic;
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

        private PhysicsSystem(Dictionary<World, PhysicsSimulatorSystem> systems)
        {
            this.systems = systems;
        }

        void ISystem.Start(in SystemContainer systemContainer, in World world)
        {
            if (systemContainer.World == world)
            {
                Dictionary<World, PhysicsSimulatorSystem> systems = new();
                systemContainer.Write(new PhysicsSystem(systems));
            }

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
            if (systemContainer.World == world)
            {
                foreach (World key in systems.Keys)
                {
                    systems[key].Dispose();
                }

                systems.Dispose();
            }
        }

        readonly unsafe uint ISystem.GetMessageHandlers(USpan<MessageHandler> buffer)
        {
            buffer[0] = MessageHandler.Create<RaycastRequest>(new(&HandleRaycast));
            return 1;
        }

        [UnmanagedCallersOnly]
        private static StatusCode HandleRaycast(SystemContainer container, World world, MemoryAddress message)
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
            return StatusCode.Continue;
        }
    }
}