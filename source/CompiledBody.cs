using BepuPhysics;
using Physics.Components;
using System;

namespace Physics.Systems
{
    public struct CompiledBody : IDisposable
    {
        public readonly uint version;
        public readonly int handle;
        public readonly BodyType type;

        public readonly BodyHandle DynamicBody => new(handle);
        public readonly StaticHandle StaticBody => new(handle);

        public CompiledBody(uint version, int handle, BodyType type)
        {
            this.version = version;
            this.handle = handle;
            this.type = type;
        }

        public readonly void Dispose()
        {

        }
    }
}