using BepuPhysics;
using Physics.Components;
using System;

namespace Physics.Systems
{
    public struct CompiledBody : IDisposable
    {
        public readonly uint version;
        public readonly int handle;
        public readonly IsBody.Type type;

        public readonly BodyHandle DynamicBody => new(handle);
        public readonly StaticHandle StaticBody => new(handle);

        public CompiledBody(uint version, int handle, IsBody.Type type)
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