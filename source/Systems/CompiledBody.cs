using BepuPhysics;
using Physics.Components;
using Simulation;
using System;
using System.Numerics;

namespace Physics.Systems
{
    public struct CompiledBody : IDisposable
    {
        public readonly int handle;
        public readonly uint version;
        public readonly eint shapeEntity;
        public readonly IsBody.Type type;

        public int shapeHash;
        public Vector3 lastPosition;
        public Quaternion lastRotation;
        public Vector3 lastScale;
        public Vector3 lastLinearVelocity;
        public Vector3 lastAngularVelocity;
        public float lastMass;

        public readonly BodyHandle DynamicBody => new(handle);
        public readonly StaticHandle StaticBody => new(handle);

        public CompiledBody(uint version, int handle, int shapeIndex, eint shapeEntity, IsBody.Type type)
        {
            this.version = version;
            this.handle = handle;
            this.shapeHash = shapeIndex;
            this.shapeEntity = shapeEntity;
            this.type = type;
        }

        public readonly void Dispose()
        {

        }
    }
}