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
            TypeRegistry.Load<PhysicsTypeBank>();
            TypeRegistry.Load<TransformsTypeBank>();
        }

        protected override void SetUp()
        {
            base.SetUp();
            simulator.AddSystem(new TransformSystem());
            simulator.AddSystem(new PhysicsSystem());
            simulator.AddSystem(new TransformSystem());
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
