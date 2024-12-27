using Physics.Components;
using Simulation.Tests;
using Transforms.Components;
using Worlds;

namespace Physics.Tests
{
    public abstract class PhysicsTests : SimulationTests
    {
        static PhysicsTests()
        {
            TypeLayout.Register<IsTransform>("IsTransform");
            TypeLayout.Register<Position>("Position");
            TypeLayout.Register<Rotation>("Rotation");
            TypeLayout.Register<Scale>("Scale");
            TypeLayout.Register<LocalToWorld>("LocalToWorld");
            TypeLayout.Register<Pivot>("Pivot");
            TypeLayout.Register<Anchor>("Anchor");
            TypeLayout.Register<WorldRotation>("WorldRotation");
            TypeLayout.Register<EulerAngles>("EulerAngles");
            TypeLayout.Register<IsBody>("IsBody");
            TypeLayout.Register<LinearVelocity>("LinearVelocity");
            TypeLayout.Register<AngularVelocity>("AngularVelocity");
            TypeLayout.Register<IsGravitySource>("IsGravitySource");
            TypeLayout.Register<IsDirectionalGravity>("IsDirectionalGravity");
            TypeLayout.Register<IsPointGravity>("IsPointGravity");
            TypeLayout.Register<GravityScale>("GravityScale");
            TypeLayout.Register<Mass>("Mass");
            TypeLayout.Register<WorldBounds>("WorldBounds");
        }

        protected override void SetUp()
        {
            base.SetUp();
            world.Schema.RegisterComponent<IsTransform>();
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
