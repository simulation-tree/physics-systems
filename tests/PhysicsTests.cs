using Physics.Components;
using Simulation.Tests;
using Transforms.Components;
using Types;

namespace Physics.Tests
{
    public abstract class PhysicsTests : SimulationTests
    {
        static PhysicsTests()
        {
            TypeLayout.Register<IsTransform>();
            TypeLayout.Register<Position>();
            TypeLayout.Register<Rotation>();
            TypeLayout.Register<Scale>();
            TypeLayout.Register<LocalToWorld>();
            TypeLayout.Register<Pivot>();
            TypeLayout.Register<Anchor>();
            TypeLayout.Register<WorldRotation>();
            TypeLayout.Register<EulerAngles>();
            TypeLayout.Register<IsBody>();
            TypeLayout.Register<LinearVelocity>();
            TypeLayout.Register<AngularVelocity>();
            TypeLayout.Register<IsGravitySource>();
            TypeLayout.Register<IsDirectionalGravity>();
            TypeLayout.Register<IsPointGravity>();
            TypeLayout.Register<GravityScale>();
            TypeLayout.Register<Mass>();
            TypeLayout.Register<WorldBounds>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            world.Schema.RegisterTag<IsTransform>();
            world.Schema.RegisterComponent<Position>();
            world.Schema.RegisterComponent<Rotation>();
            world.Schema.RegisterComponent<Scale>();
            world.Schema.RegisterComponent<LocalToWorld>();
            world.Schema.RegisterComponent<Pivot>();
            world.Schema.RegisterComponent<Anchor>();
            world.Schema.RegisterComponent<WorldRotation>();
            world.Schema.RegisterComponent<EulerAngles>();
            world.Schema.RegisterComponent<IsBody>();
            world.Schema.RegisterComponent<LinearVelocity>();
            world.Schema.RegisterComponent<AngularVelocity>();
            world.Schema.RegisterComponent<IsGravitySource>();
            world.Schema.RegisterComponent<IsDirectionalGravity>();
            world.Schema.RegisterComponent<IsPointGravity>();
            world.Schema.RegisterComponent<GravityScale>();
            world.Schema.RegisterComponent<Mass>();
            world.Schema.RegisterComponent<WorldBounds>();
        }
    }
}
