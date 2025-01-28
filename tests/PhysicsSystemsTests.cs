using Physics.Systems;
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
            simulator.AddSystem<TransformSystem>();
            simulator.AddSystem<PhysicsSystem>();
            simulator.AddSystem<TransformSystem>();
        }

        protected override Schema CreateSchema()
        {
            Schema schema = base.CreateSchema();
            schema.Load<PhysicsSchemaBank>();
            schema.Load<TransformsSchemaBank>();
            return schema;
        }
    }
}
