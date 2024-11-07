using Physics.Components;
using Physics.Events;
using Physics.Systems;
using Simulation;
using Simulation.Tests;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Transforms;
using Transforms.Systems;
using Unmanaged;

namespace Physics.Tests
{
    public class RaycastTests : SimulationTests
    {
        protected override void SetUp()
        {
            base.SetUp();
            Simulator.AddSystem<TransformSystem>();
            Simulator.AddSystem<PhysicsSystem>();
            Simulator.AddSystem<TransformSystem>();
        }

        [Test]
        public unsafe void RaycastWithResults()
        {
            Body cubeBody = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            Transform cubeTransform = cubeBody.transform;

            cubeTransform.LocalPosition = new(0, 0, 5);
            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Simulator.TryHandleMessage(new RaycastRequest(World, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 0));
            Assert.That(FindProof(0), Is.True);

            cubeTransform.LocalPosition = new(0, 0, 10);
            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Simulator.TryHandleMessage(new RaycastRequest(World, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 1));
            Assert.That(FindProof(1), Is.True);

            cubeTransform.LocalPosition = new(5, 0, 0);
            Simulator.Update(TimeSpan.FromSeconds(0.01f));

            Simulator.TryHandleMessage(new RaycastRequest(World, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 2));
            Assert.That(FindProof(2), Is.True);

            bool FindProof(ulong identifier)
            {
                bool found = false;
                World.ForEach((in uint entity, ref bool boolean, ref ulong proofIdentifier) =>
                {
                    if (identifier == proofIdentifier)
                    {
                        found = true;
                    }
                });

                return found;
            }

            [UnmanagedCallersOnly]
            static void HitCallback(World world, RaycastRequest raycast, RaycastHit* hitsPointer, uint hitsLength)
            {
                uint proofEntity = world.CreateEntity();
                world.AddComponent(proofEntity, true);
                world.AddComponent(proofEntity, raycast.identifier);

                USpan<RaycastHit> hits = new(hitsPointer, hitsLength);
                if (raycast.identifier == 0)
                {
                    Assert.That(hits.Length, Is.EqualTo(1));
                    RaycastHit hit = hits[0];
                    Assert.That(hit.distance, Is.EqualTo(4.5f).Within(0.1f));
                }
                else if (raycast.identifier == 1)
                {
                    Assert.That(hits.Length, Is.EqualTo(1));
                    RaycastHit hit = hits[0];
                    Assert.That(hit.distance, Is.EqualTo(9.5f).Within(0.1f));
                }
                else if (raycast.identifier == 2)
                {
                    Assert.That(hits.Length, Is.EqualTo(0));
                }
                else
                {
                    Assert.Fail();
                }
            }
        }
    }
}
