using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;

namespace Physics.Systems
{
    public struct CompiledShape : IDisposable
    {
        public readonly Vector3 offset;
        public readonly Vector3 scale;
        public readonly TypedIndex shapeIndex;
        public readonly BodyInertia bodyInertia;

        public CompiledShape(Vector3 offset, Vector3 scale, TypedIndex shapeIndex, BodyInertia bodyInertia)
        {
            this.offset = offset;
            this.scale = scale;
            this.shapeIndex = shapeIndex;
            this.bodyInertia = bodyInertia;
        }

        public readonly void Dispose()
        {
        }
    }
}