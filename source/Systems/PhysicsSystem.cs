using Collections;
using Physics.Events;
using Simulation;
using Simulation.Functions;
using System;
using System.Runtime.InteropServices;
using Unmanaged;
using Worlds;

namespace Physics.Systems
{
    public readonly struct PhysicsSystem : ISystem
    {
        private readonly Dictionary<World, PhysicsSimulatorSystem> systems;

        readonly unsafe StartSystem ISystem.Start => new(&Start);
        readonly unsafe UpdateSystem ISystem.Update => new(&Update);
        readonly unsafe FinishSystem ISystem.Finish => new(&Finish);

        readonly unsafe uint ISystem.GetMessageHandlers(USpan<MessageHandler> buffer)
        {
            buffer[0] = MessageHandler.Create<RaycastRequest>(new(&HandleRaycast));
            return 1;
        }

        [UnmanagedCallersOnly]
        private static void Start(SystemContainer container, World world)
        {
            ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
            if (!system.systems.ContainsKey(world))
            {
                system.systems.TryAdd(world, new PhysicsSimulatorSystem(world));
            }
        }

        [UnmanagedCallersOnly]
        private static void Update(SystemContainer container, World world, TimeSpan delta)
        {
            ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
            ref PhysicsSimulatorSystem physicsSystem = ref system.systems.TryGetValue(world, out bool contains);
            if (!contains)
            {
                physicsSystem = new(world);
                system.systems.TryAdd(world, physicsSystem);
            }

            physicsSystem.Update(delta);
        }

        [UnmanagedCallersOnly]
        private static void Finish(SystemContainer container, World world)
        {
            if (container.World == world)
            {
                ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
                system.CleanUp();
            }
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
                system.systems.TryAdd(world, physicsSystem);
            }

            physicsSystem.PerformRaycastRequest(raycast);
        }

        public PhysicsSystem()
        {
            systems = new();
        }

        private readonly void CleanUp()
        {
            foreach (World world in systems.Keys)
            {
                systems[world].Dispose();
            }

            systems.Dispose();
        }
    }
}