using Physics.Events;
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
            static void HitCallback(World world, RaycastRequest raycast, RaycastHit* hitsPointer, uint hitsLength)
            {
                uint proofEntity = world.CreateEntity();
                world.AddComponent(proofEntity, true);
                world.AddComponent(proofEntity, raycast.userData);

                USpan<RaycastHit> hits = new(hitsPointer, hitsLength);
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
