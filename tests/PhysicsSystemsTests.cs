using Physics.Messages;
using Physics.Systems;
using Physics.Systems.Tests;
using Simulation.Tests;
using Transforms;
using Transforms.Messages;
using Transforms.Systems;
using Types;
using Worlds;

namespace Physics.Tests
{
    public abstract class PhysicsSystemsTests : SimulationTests
    {
        public World world;

        static PhysicsSystemsTests()
        {
            MetadataRegistry.Load<PhysicsMetadataBank>();
            MetadataRegistry.Load<TransformsMetadataBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            Schema schema = new();
            schema.Load<PhysicsSchemaBank>();
            schema.Load<TransformsSchemaBank>();
            schema.Load<PhysicsSystemsTestsSchemaBank>();
            world = new(schema);
            Simulator.Add(new TransformSystem(Simulator, world));
            Simulator.Add(new PhysicsSystem(Simulator, world));
        }

        protected override void TearDown()
        {
            Simulator.Remove<PhysicsSystem>();
            Simulator.Remove<TransformSystem>();
            world.Dispose();
            base.TearDown();
        }

        protected override void Update(double deltaTime)
        {
            Simulator.Broadcast(new TransformUpdate());
            Simulator.Broadcast(new PhysicsUpdate(deltaTime));
            Simulator.Broadcast(new TransformUpdate());
        }
    }
}
