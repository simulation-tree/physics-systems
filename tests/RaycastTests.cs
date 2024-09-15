using Physics.Components;
using Physics.Events;
using Physics.Systems;
using Simulation;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Transforms;
using Transforms.Events;
using Transforms.Systems;
using Unmanaged;

namespace Physics.Tests
{
    public class RaycastTests
    {
        [TearDown]
        public void CleanUp()
        {
            Allocations.ThrowIfAny();
        }

        private void Simulate(World world, TimeSpan delta)
        {
            world.Submit(new TransformUpdate());
            world.Submit(new PhysicsUpdate(delta));
            world.Submit(new TransformUpdate());
            world.Poll();
        }

        [Test]
        public unsafe void RaycastWithResults()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            Body cubeBody = new(world, new CubeShape(0.5f), IsBody.Type.Dynamic);
            Transform cubeTransform = cubeBody.transform;
            cubeTransform.LocalPosition = new(0, 0, 5);

            world.Submit(new Raycast(Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 0));
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            cubeTransform.LocalPosition = new(0, 0, 10);

            world.Submit(new Raycast(Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 1));
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            cubeTransform.LocalPosition = new(5, 0, 0);

            world.Submit(new Raycast(Vector3.Zero, Vector3.UnitZ, new(&HitCallback), 100f, 2));
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            [UnmanagedCallersOnly]
            static void HitCallback(World world, Raycast raycast, RaycastHit* hitsPointer, uint hitsLength)
            {
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
