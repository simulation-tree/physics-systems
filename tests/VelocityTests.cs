using Physics.Components;
using Physics.Systems;
using System;
using System.Numerics;
using Transforms;
using Transforms.Systems;

namespace Physics.Tests
{
    public class VelocityTests : PhysicsTests
    {
        protected override void SetUp()
        {
            base.SetUp();
            Simulator.AddSystem<TransformSystem>();
            Simulator.AddSystem<PhysicsSystem>();
            Simulator.AddSystem<TransformSystem>();
        }

        [Test]
        public void PushRockForward()
        {
            Body rock = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            rock.LinearVelocity = new(1, 0, 0);

            Simulator.Update(TimeSpan.FromSeconds(1f));

            Transform rockTransform = rock;
            Assert.That(rockTransform.WorldPosition.X, Is.EqualTo(1f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));

            Simulator.Update(TimeSpan.FromSeconds(1f));

            Assert.That(rockTransform.WorldPosition.X, Is.EqualTo(2f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void FreeFall2Seconds()
        {
            Body rock = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            DirectionalGravity directionalGravity = new(World, -Vector3.UnitY);
            Transform rockTransform = rock;

            Simulator.Update(TimeSpan.FromSeconds(1f));

            Console.WriteLine(rockTransform.WorldPosition);
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-9.806f).Within(0.1f));

            Simulator.Update(TimeSpan.FromSeconds(1f));

            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-29.42f).Within(0.2f));

            Body otherRock = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            Transform otherRockTransform = otherRock;
            otherRockTransform.LocalPosition = new(2, 0, 0);

            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            //Simulate(world, TimeSpan.FromSeconds(0.2f));
            Simulator.Update(TimeSpan.FromSeconds(1f));

            Assert.That(otherRockTransform.WorldPosition.Y, Is.EqualTo(-9.806f).Within(0.1f));

            Simulator.Update(TimeSpan.FromSeconds(1f));

            Assert.That(otherRockTransform.WorldPosition.Y, Is.EqualTo(-29.42f).Within(0.2f));
        }

        [Test]
        public void ThrowRockAgainstGravity()
        {
            Body rock = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            rock.LinearVelocity = new(4, 4, 0);

            DirectionalGravity directionalGravity = new(World, -Vector3.UnitY);

            Simulator.Update(TimeSpan.FromSeconds(0.2f));
            Simulator.Update(TimeSpan.FromSeconds(0.2f));
            Simulator.Update(TimeSpan.FromSeconds(0.2f));
            Simulator.Update(TimeSpan.FromSeconds(0.2f));
            Simulator.Update(TimeSpan.FromSeconds(0.2f));

            Transform rockTransform = rock;
            Assert.That(rockTransform.WorldPosition.X, Is.EqualTo(4f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Y, Is.EqualTo(-1.884f).Within(0.1f));
            Assert.That(rockTransform.WorldPosition.Z, Is.EqualTo(0).Within(0.1f));
        }

        [Test]
        public void BallFallingOntoStaticFloor()
        {
            DirectionalGravity directionalGravity = new(World, -Vector3.UnitY);

            Body ball = new(World, new SphereShape(0.5f), IsBody.Type.Dynamic);
            Transform ballTransform = ball;
            ballTransform.LocalPosition = new(0f, 5f, 0f);

            Body floor = new(World, new CubeShape(5f, 0.5f, 5f), IsBody.Type.Static);

            Simulator.Update(TimeSpan.FromSeconds(4f));

            Assert.That(ballTransform.WorldPosition.Y, Is.EqualTo(1f).Within(0.1f));

            Transform floorTransform = floor;
            Assert.That(floorTransform.WorldPosition.X, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Y, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void KinematicMovingPlatform()
        {
            DirectionalGravity directionalGravity = new(World, -Vector3.UnitY);

            Body ball = new(World, new SphereShape(0.5f), IsBody.Type.Dynamic);
            Transform ballTransform = ball;
            ballTransform.LocalPosition = new(0f, 5f, 0f);

            Body platform = new(World, new CubeShape(5f, 0.5f, 5f), IsBody.Type.Kinematic, new(0, 1, 0)); //platform moves up

            Simulator.Update(TimeSpan.FromSeconds(4f));

            Assert.That(ballTransform.WorldPosition.Y, Is.EqualTo(5f).Within(0.1f));

            Transform floorTransform = platform;
            Assert.That(floorTransform.WorldPosition.X, Is.EqualTo(0f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Y, Is.EqualTo(4f).Within(0.1f));
            Assert.That(floorTransform.WorldPosition.Z, Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void CheckCalculatedBounds()
        {
            Body cubeBody = new(World, new CubeShape(0.5f), IsBody.Type.Dynamic);
            Simulator.Update(TimeSpan.FromSeconds(0.1f));

            (Vector3 min, Vector3 max) = cubeBody.Bounds;
            Assert.That(min, Is.EqualTo(new Vector3(-0.5f, -0.5f, -0.5f)));
            Assert.That(max, Is.EqualTo(new Vector3(0.5f, 0.5f, 0.5f)));

            cubeBody.Shape.offset = new(1, 1, 1);
            Simulator.Update(TimeSpan.FromSeconds(0.1f));

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
            Simulator.Update(TimeSpan.FromSeconds(0.1f));

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
}
