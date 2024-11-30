using BepuUtilities.Memory;
using System;
using System.Runtime.InteropServices;

namespace Physics.Systems
{
    public readonly struct BepuBufferPool : IDisposable
    {
        private readonly GCHandle handle;

        public BepuBufferPool()
        {
            BufferPool bufferPool = new();
            handle = GCHandle.Alloc(bufferPool);
        }

        public readonly void Dispose()
        {
            BufferPool bufferPool = this;
            bufferPool.Clear();
            handle.Free();
        }

        public static implicit operator BufferPool(BepuBufferPool bufferPool)
        {
            return (BufferPool)(bufferPool.handle.Target ?? throw new ObjectDisposedException("BepuBufferPool"));
        }
    }
}