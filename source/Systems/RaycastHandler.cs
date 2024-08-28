using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using System.Numerics;
using Unmanaged.Collections;

namespace Physics.Systems
{
    public readonly struct RaycastHandler : IRayHitHandler
    {
        private readonly UnmanagedList<(bool, int, float, Vector3)> hits;

        public RaycastHandler(UnmanagedList<(bool, int, float, Vector3)> hits)
        {
            this.hits = hits;
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
            if (collidable.Mobility == CollidableMobility.Dynamic || collidable.Mobility == CollidableMobility.Kinematic)
            {
                hits.Add((false, collidable.BodyHandle.Value, t, normal));
            }
            else
            {
                hits.Add((true, collidable.StaticHandle.Value, t, normal));
            }
        }
    }
}