using Physics.Components;
using Physics.Events;
using Physics.Systems;
using Simulation;
using System;
using System.Numerics;
using Transforms;
using Transforms.Events;
using Transforms.Systems;
using Unmanaged;

namespace Physics.Tests
{
    public class VelocityTests
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
        public void PushRockForward1Second()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            CubeShape cubeShape = new(world, 0.5f);
            Body rock = new(world, cubeShape, IsBody.Type.Dynamic);
            rock.Velocity = new(1, 0, 0);

            Simulate(world, TimeSpan.FromSeconds(1f));

            Transform rockTransform = rock;
            Assert.That(rockTransform.WorldPosition.X, Is.EqualTo(1f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void FreeFall2Seconds()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            CubeShape cubeShape = new(world, 0.5f);
            Body rock = new(world, cubeShape, IsBody.Type.Dynamic);
            DirectionalGravity directionalGravity = new(world, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI * 0.5f));
            Transform rockTransform = rock;

            Simulate(world, TimeSpan.FromSeconds(1f));

            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-9.806f).Within(0.1f));

            Simulate(world, TimeSpan.FromSeconds(1f));

            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-29.42f).Within(0.2f));

            Body otherRock = new(world, cubeShape, IsBody.Type.Dynamic);
            Transform otherRockTransform = otherRock;
            otherRockTransform.LocalPosition = new(2, 0, 0);

            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulate(world, TimeSpan.FromSeconds(1f));

            Assert.That(otherRockTransform.WorldPosition.Y, Is.EqualTo(-9.806f).Within(0.1f));

            Simulate(world, TimeSpan.FromSeconds(1f));

            Assert.That(otherRockTransform.WorldPosition.Y, Is.EqualTo(-29.42f).Within(0.2f));
        }

        [Test]
        public void ThrowRockAgainstGravity()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            CubeShape cubeShape = new(world, 0.5f);
            Body rock = new(world, cubeShape, IsBody.Type.Dynamic);
            rock.Velocity = new(4, 4, 0);

            DirectionalGravity directionalGravity = new(world, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI * 0.5f));

            Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulate(world, TimeSpan.FromSeconds(0.2f));

            Transform rockTransform = rock;
            Assert.That(rockTransform.WorldPosition.X, Is.EqualTo(4f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-1.884f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Z, Is.EqualTo(0).Within(0.1f));
        }

        [Test]
        public void BallFallingOntoStaticFloor()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            DirectionalGravity directionalGravity = new(world, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI * 0.5f));
            SphereShape sphereShape = new(world, 0.5f);
            Body ball = new(world, sphereShape, IsBody.Type.Dynamic);
            Transform ballTransform = ball;
            ballTransform.LocalPosition = new(0f, 5f, 0f);
            CubeShape floorShape = new(world, 5f, 0.5f, 5f);
            Body floor = new(world, floorShape, IsBody.Type.Static);

            Simulate(world, TimeSpan.FromSeconds(4f));

            Assert.That(ballTransform.WorldPosition.Y, Is.EqualTo(1f).Within(0.1f));

            Transform floorTransform = floor;
            Assert.That(floorTransform.WorldPosition.X, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void KinematicMovingPlatform()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            DirectionalGravity directionalGravity = new(world, Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI * 0.5f));
            SphereShape sphereShape = new(world, 0.5f);
            Body ball = new(world, sphereShape, IsBody.Type.Dynamic);
            Transform ballTransform = ball;
            ballTransform.LocalPosition = new(0f, 5f, 0f);

            CubeShape platformShape = new(world, 5f, 0.5f, 5f);
            Body platform = new(world, platformShape, IsBody.Type.Kinematic, new(0, 1, 0)); //platform moves up

            Simulate(world, TimeSpan.FromSeconds(4f));

            Assert.That(ballTransform.WorldPosition.Y, Is.EqualTo(5f).Within(0.1f));

            Transform floorTransform = platform;
            Assert.That(floorTransform.WorldPosition.X, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Y, Is.EqualTo(4f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void CheckCalculatedBounds()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            CubeShape cubeShape = new(world, 0.5f);
            Body cubeBody = new(world, cubeShape, IsBody.Type.Dynamic);
            Entity cubeEntity = cubeBody;
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            (Vector3 min, Vector3 max) = cubeBody.Bounds;
            Assert.That(min, Is.EqualTo(new Vector3(-0.5f, -0.5f, -0.5f)));
            Assert.That(max, Is.EqualTo(new Vector3(0.5f, 0.5f, 0.5f)));

            cubeShape.Offset = new(1, 1, 1);
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            (min, max) = cubeBody.Bounds;
            Vector3 size = max - min;
            Vector3 center = (max + min) * 0.5f;
            Assert.That(size.X, Is.EqualTo(1f).Within(0.01f));
            Assert.That(size.Y, Is.EqualTo(1f).Within(0.01f));
            Assert.That(size.Z, Is.EqualTo(1f).Within(0.01f));
            Assert.That(min.X, Is.EqualTo(0.5f).Within(0.01f));
            Assert.That(min.Y, Is.EqualTo(0.5f).Within(0.01f));
            Assert.That(min.Z, Is.EqualTo(0.5f).Within(0.01f));
            Assert.That(center.X, Is.EqualTo(1f).Within(0.01f));
            Assert.That(center.Y, Is.EqualTo(1f).Within(0.01f));
            Assert.That(center.Z, Is.EqualTo(1f).Within(0.01f));

            //45 degrees on y axis
            Quaternion rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.25f);
            Transform cubeTransform = cubeBody;
            cubeTransform.WorldRotation = rotation;
            Simulate(world, TimeSpan.FromSeconds(0.1f));

            (min, max) = cubeBody.Bounds;
            size = max - min;
            center = (max + min) * 0.5f;
            Assert.That(size.X, Is.EqualTo(1.4142f).Within(0.01f));
            Assert.That(size.Y, Is.EqualTo(1f).Within(0.01f));
            Assert.That(size.Z, Is.EqualTo(1.4142f).Within(0.01f));
            Assert.That(min.X, Is.EqualTo(0.7071f).Within(0.01f));
            Assert.That(min.Y, Is.EqualTo(0.5f).Within(0.01f));
            Assert.That(min.Z, Is.EqualTo(-0.7071f).Within(0.01f));
            Assert.That(center.X, Is.EqualTo(1.4142f).Within(0.01f));
            Assert.That(center.Y, Is.EqualTo(1f).Within(0.01f));
            Assert.That(center.Z, Is.EqualTo(0f).Within(0.01f));
        }
    }

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
        public void RaycastWithResults()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);

            CubeShape cubeShape = new(world, 0.5f);
            Body cubeBody = new(world, cubeShape, IsBody.Type.Dynamic);
            Transform cubeTransform = cubeBody;
            cubeTransform.LocalPosition = new(0, 0, 5);

            Raycaster ray = new(world, new(0, 0, 0), Quaternion.Identity);

            Simulate(world, TimeSpan.FromSeconds(0.1f));

            ReadOnlySpan<RaycastHit> hits = ray.Hits;
            Assert.That(hits.Length, Is.EqualTo(1));
            RaycastHit hit = hits[0];
            Assert.That(hit.distance, Is.EqualTo(4.5f).Within(0.1f));

            cubeTransform.LocalPosition = new(0, 0, 10);

            Simulate(world, TimeSpan.FromSeconds(0.1f));

            hits = ray.Hits;
            Assert.That(hits.Length, Is.EqualTo(1));
            hit = hits[0];
            Assert.That(hit.distance, Is.EqualTo(9.5f).Within(0.1f));

            cubeTransform.LocalPosition = new(5, 0, 0);

            Simulate(world, TimeSpan.FromSeconds(0.1f));

            hits = ray.Hits;
            Assert.That(hits.Length, Is.EqualTo(0));
        }

        [Test]
        public void DontPerformDisabledRaycasts()
        {
            using World world = new();
            using TransformSystem transforms = new(world);
            using PhysicsSystem physics = new(world);
            CubeShape cubeShape = new(world, 0.5f);
            Body cubeBody = new(world, cubeShape, IsBody.Type.Dynamic);
            Transform cubeTransform = cubeBody;
            cubeTransform.LocalPosition = new(0, 0, 5);
            Raycaster ray = new(world, new(0, 0, 0), Quaternion.Identity);
            ray.IsEnabled = false;

            Simulate(world, TimeSpan.FromSeconds(0.1f));

            ReadOnlySpan<RaycastHit> hits = ray.Hits;
            Assert.That(hits.Length, Is.EqualTo(0));
        }
    }
}
