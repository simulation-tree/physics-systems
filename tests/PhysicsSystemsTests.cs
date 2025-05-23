using Physics.Systems;
using Physics.Systems.Tests;
using Simulation.Tests;
using Transforms;
using Transforms.Systems;
using Types;
using Worlds;

namespace Physics.Tests
{
    public abstract class PhysicsSystemsTests : SimulationTests
    {
        static PhysicsSystemsTests()
        {
            MetadataRegistry.Load<PhysicsMetadataBank>();
            MetadataRegistry.Load<TransformsMetadataBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            simulator.Add(new TransformSystem());
            simulator.Add(new PhysicsSystem(simulator));
            simulator.Add(new TransformSystem());
        }

        protected override void TearDown()
        {
            simulator.Remove<TransformSystem>();
            simulator.Remove<PhysicsSystem>();
            simulator.Remove<TransformSystem>();
            base.TearDown();
        }

        protected override Schema CreateSchema()
        {
            Schema schema = base.CreateSchema();
            schema.Load<PhysicsSchemaBank>();
            schema.Load<TransformsSchemaBank>();
            schema.Load<PhysicsSystemsTestsSchemaBank>();
            return schema;
        }
    }
}
