using System.Numerics;

namespace Physics.Systems
{
    internal readonly struct PointGravitySource
    {
        public readonly Vector3 position;
        public readonly float force;
        public readonly float radius;
        public readonly float radiusSquared;

        public PointGravitySource(Vector3 position, float force, float radius)
        {
            this.position = position;
            this.force = force;
            this.radius = radius;
            radiusSquared = radius * radius;
        }
    }
}