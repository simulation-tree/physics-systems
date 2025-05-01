using Physics.Events;
using Physics.Functions;
using Shapes.Types;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Transforms;
using Unmanaged;
using Worlds;

namespace Physics.Tests
{
    public class RaycastTests : PhysicsSystemsTests
    {
        [Test]
        public unsafe void RaycastAgainstStaticFloor()
        {
            Body floorBody = new(world, new CubeShape(0.5f, 0.5f, 0.5f), BodyType.Static);
            floorBody.As<Transform>().LocalPosition = new(0f, -5f, 0f);
            floorBody.As<Transform>().LocalScale = new(100f, 1f, 1f);

            simulator.Update(TimeSpan.FromSeconds(0.01f));

            Vector3 origin = new(0.00013398135f, -4.000277f, 0.00049429137f);
            Vector3 direction = -Vector3.UnitY;
            float distance = 0.5f;
            using MemoryAddress temp = MemoryAddress.AllocateValue<(uint, bool)>(default);
            ref (uint entity, bool hit) result = ref temp.Read<(uint, bool)>();
            Assert.That(result.hit, Is.False);
            simulator.TryHandleMessage(new RaycastRequest(world, origin, direction, new(&HitCallback), distance, (ulong)temp.Address));
            Assert.That(result.hit, Is.True);

            [UnmanagedCallersOnly]
            static void HitCallback(RaycastHitCallback.Input input)
            {
                World world = input.world;
                RaycastRequest raycast = input.request;
                MemoryAddress temp = new((void*)raycast.userData);
                ref (uint entity, bool hit) result = ref temp.Read<(uint, bool)>();
                ReadOnlySpan<RaycastHit> hits = input.Hits;
                if (hits.Length == 0)
                {
                    result.hit = false;
                    return;
                }

                RaycastHit hit = hits[0];
                result.hit = true;
                result.entity = hit.entity;
            }
        }

        [Test]
        public unsafe void RaycastWithResults()
        {
            Body cubeBody = new(world, new CubeShape(0.5f), BodyType.Dynamic);
            Transform cubeTransform = cubeBody;

            cubeTransform.LocalPosition = new(0, 0, 5);
            simulator.Update(TimeSpan.FromSeconds(0.01f));

            simulator.TryHandleMessage(new RaycastRequest(world, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 0));
            Assert.That(FindProof(0), Is.True);

            cubeTransform.LocalPosition = new(0, 0, 10);
            simulator.Update(TimeSpan.FromSeconds(0.01f));

            simulator.TryHandleMessage(new RaycastRequest(world, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 1));
            Assert.That(FindProof(1), Is.True);

            cubeTransform.LocalPosition = new(5, 0, 0);
            simulator.Update(TimeSpan.FromSeconds(0.01f));

            simulator.TryHandleMessage(new RaycastRequest(world, Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 2));
            Assert.That(FindProof(2), Is.True);

            bool FindProof(ulong userData)
            {
                ComponentQuery<bool, ulong> query = new(world);
                foreach (var q in query)
                {
                    ref ulong id = ref q.component2;
                    if (id == userData)
                    {
                        return true;
                    }
                }

                return false;
            }

            [UnmanagedCallersOnly]
            static void HitCallback(RaycastHitCallback.Input input)
            {
                World world = input.world;
                RaycastRequest raycast = input.request;

                uint proofEntity = world.CreateEntity();
                world.AddComponent(proofEntity, true);
                world.AddComponent(proofEntity, raycast.userData);

                ReadOnlySpan<RaycastHit> hits = input.Hits;
                if (raycast.userData == 0)
                {
                    Assert.That(hits.Length, Is.EqualTo(1));
                    RaycastHit hit = hits[0];
                    Assert.That(hit.distance, Is.EqualTo(4.5f).Within(0.1f));
                }
                else if (raycast.userData == 1)
                {
                    Assert.That(hits.Length, Is.EqualTo(1));
                    RaycastHit hit = hits[0];
                    Assert.That(hit.distance, Is.EqualTo(9.5f).Within(0.1f));
                }
                else if (raycast.userData == 2)
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
