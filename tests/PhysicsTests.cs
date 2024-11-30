using Physics.Components;
using Simulation.Tests;
using Transforms.Components;
using Worlds;

namespace Physics.Tests
{
    public abstract class PhysicsTests : SimulationTests
    {
        protected override void SetUp()
        {
            base.SetUp();
            ComponentType.Register<IsTransform>();
            ComponentType.Register<Position>();
            ComponentType.Register<Rotation>();
            ComponentType.Register<Scale>();
            ComponentType.Register<LocalToWorld>();
            ComponentType.Register<Pivot>();
            ComponentType.Register<Anchor>();
            ComponentType.Register<WorldRotation>();
            ComponentType.Register<EulerAngles>();
            ComponentType.Register<IsBody>();
            ComponentType.Register<LinearVelocity>();
            ComponentType.Register<AngularVelocity>();
            ComponentType.Register<IsGravitySource>();
            ComponentType.Register<IsDirectionalGravity>();
            ComponentType.Register<IsPointGravity>();
            ComponentType.Register<GravityScale>();
            ComponentType.Register<Mass>();
            ComponentType.Register<WorldBounds>();
        }
    }
}
