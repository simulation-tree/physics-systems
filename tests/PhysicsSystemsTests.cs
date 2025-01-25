using Physics.Systems;
using Simulation.Tests;
using Transforms.Systems;
using Types;
using Worlds;

namespace Physics.Tests
{
    public abstract class PhysicsSystemsTests : SimulationTests
    {
        static PhysicsSystemsTests()
        {
            TypeRegistry.Load<Physics.TypeBank>();
            TypeRegistry.Load<Transforms.TypeBank>();
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
            schema.Load<Physics.SchemaBank>();
            schema.Load<Transforms.SchemaBank>();
            return schema;
        }
    }
}
