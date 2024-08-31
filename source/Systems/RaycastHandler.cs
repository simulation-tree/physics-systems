using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using System.Numerics;
using Unmanaged.Collections;

namespace Physics.Systems
{
    public readonly struct RaycastHandler : IRayHitHandler
    {
        private readonly UnmanagedList<RaycastHit> hits;
        private readonly PhysicsSystem system;

        public RaycastHandler(UnmanagedList<RaycastHit> hits, PhysicsSystem system)
        {
            this.hits = hits;
            this.system = system;
        }

        bool IRayHitHandler.AllowTest(CollidableReference collidable)
        {
            return true;
        }

        bool IRayHitHandler.AllowTest(CollidableReference collidable, int childIndex)
        {
            return true;
        }

        void IRayHitHandler.OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, CollidableReference collidable, int childIndex)
        {
            bool isStatic = collidable.Mobility == CollidableMobility.Static;
            uint hitEntity = system.GetPhysicsEntity(collidable.RawHandleValue, isStatic);
            Vector3 point = ray.Origin + ray.Direction * t;
            hits.Add(new RaycastHit(point, normal, t, hitEntity));
        }
    }
}