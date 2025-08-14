using System.Numerics;

namespace Physics.Systems
{
    public struct PhysicsObjectState
    {
        public bool valid;
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;

        public PhysicsObjectState(Vector3 position, Quaternion rotation, Vector3 linearVelocity, Vector3 angularVelocity)
        {
            valid = true;
            this.position = position;
            this.rotation = rotation;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
        }
    }
}