namespace Physics.Tests
{
    public class ShapeTypeTests
    {
        [Test]
        public void CreateCubeShape()
        {
            CubeShape cubeShape = new(1f, 2f, 3f);
            Shape shape = Shape.Create(cubeShape);
            Assert.That(shape.Is<CubeShape>(), Is.True);
            Assert.That(shape.Read<CubeShape>(), Is.EqualTo(cubeShape));
        }

        [Test]
        public void CreateSphereShape()
        {
            SphereShape sphereShape = new(1f);
            Shape shape = Shape.Create(sphereShape);
            Assert.That(shape.Is<SphereShape>(), Is.True);
            Assert.That(shape.Read<SphereShape>(), Is.EqualTo(sphereShape));
        }
    }
}
