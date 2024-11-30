using System.Numerics;

namespace Physics.Systems
{
    public struct BodyState
    {
        public Vector3 position;
        public Quaternion rotation;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;

        public BodyState(Vector3 position, Quaternion rotation, Vector3 linearVelocity, Vector3 angularVelocity)
        {
            this.position = position;
            this.rotation = rotation;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
        }
    }
}