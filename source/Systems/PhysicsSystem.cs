using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using Physics.Components;
using Physics.Events;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Unmanaged.Collections;

namespace Physics.Systems
{
    public struct DemoNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public SpringSettings ContactSpringiness;
        public float MaximumRecoveryVelocity;
        public float FrictionCoefficient;

        public DemoNarrowPhaseCallbacks(SpringSettings contactSpringiness, float maximumRecoveryVelocity = 2f, float frictionCoefficient = 1f)
        {
            ContactSpringiness = contactSpringiness;
            MaximumRecoveryVelocity = maximumRecoveryVelocity;
            FrictionCoefficient = frictionCoefficient;
        }

        public void Initialize(BepuPhysics.Simulation simulation)
        {
            //Use a default if the springiness value wasn't initialized... at least until struct field initializers are supported outside of previews.
            if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
            {
                ContactSpringiness = new(30, 1);
                MaximumRecoveryVelocity = 2f;
                FrictionCoefficient = 1f;
            }
        }

        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            //While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
            //Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
            //to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
            return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        }

        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = FrictionCoefficient;
            pairMaterial.MaximumRecoveryVelocity = MaximumRecoveryVelocity;
            pairMaterial.SpringSettings = ContactSpringiness;
            return true;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }

        public void Dispose()
        {
        }
    }

    public struct DemoPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        /// <summary>
        /// Gravity to apply to dynamic bodies in the simulation.
        /// </summary>
        private Allocation gravity;
        /// <summary>
        /// Fraction of dynamic body linear velocity to remove per unit of time. Values range from 0 to 1. 0 is fully undamped, while values very close to 1 will remove most velocity.
        /// </summary>
        public float LinearDamping;
        /// <summary>
        /// Fraction of dynamic body angular velocity to remove per unit of time. Values range from 0 to 1. 0 is fully undamped, while values very close to 1 will remove most velocity.
        /// </summary>
        public float AngularDamping;


        /// <summary>
        /// Gets how the pose integrator should handle angular velocity integration.
        /// </summary>
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        /// <summary>
        /// Gets whether the integrator should use substepping for unconstrained bodies when using a substepping solver.
        /// If true, unconstrained bodies will be integrated with the same number of substeps as the constrained bodies in the solver.
        /// If false, unconstrained bodies use a single step of length equal to the dt provided to Simulation.Timestep. 
        /// </summary>
        public readonly bool AllowSubstepsForUnconstrainedBodies => false;

        /// <summary>
        /// Gets whether the velocity integration callback should be called for kinematic bodies.
        /// If true, IntegrateVelocity will be called for bundles including kinematic bodies.
        /// If false, kinematic bodies will just continue using whatever velocity they have set.
        /// Most use cases should set this to false.
        /// </summary>
        public readonly bool IntegrateVelocityForKinematics => false;

        public void Initialize(BepuPhysics.Simulation simulation)
        {
            //In this demo, we don't need to initialize anything.
            //If you had a simulation with per body gravity stored in a CollidableProperty<T> or something similar, having the simulation provided in a callback can be helpful.
        }

        /// <summary>
        /// Creates a new set of simple callbacks for the demos.
        /// </summary>
        /// <param name="gravity">Gravity to apply to dynamic bodies in the simulation.</param>
        /// <param name="linearDamping">Fraction of dynamic body linear velocity to remove per unit of time. Values range from 0 to 1. 0 is fully undamped, while values very close to 1 will remove most velocity.</param>
        /// <param name="angularDamping">Fraction of dynamic body angular velocity to remove per unit of time. Values range from 0 to 1. 0 is fully undamped, while values very close to 1 will remove most velocity.</param>
        public DemoPoseIntegratorCallbacks(Allocation gravity, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            this.gravity = gravity;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
        }

        Vector3Wide gravityWideDt;
        Vector<float> linearDampingDt;
        Vector<float> angularDampingDt;

        /// <summary>
        /// Callback invoked ahead of dispatches that may call into <see cref="IntegrateVelocity"/>.
        /// It may be called more than once with different values over a frame. For example, when performing bounding box prediction, velocity is integrated with a full frame time step duration.
        /// During substepped solves, integration is split into substepCount steps, each with fullFrameDuration / substepCount duration.
        /// The final integration pass for unconstrained bodies may be either fullFrameDuration or fullFrameDuration / substepCount, depending on the value of AllowSubstepsForUnconstrainedBodies. 
        /// </summary>
        /// <param name="dt">Current integration time step duration.</param>
        /// <remarks>This is typically used for precomputing anything expensive that will be used across velocity integration.</remarks>
        public void PrepareForIntegration(float dt)
        {
            //No reason to recalculate gravity * dt for every body; just cache it ahead of time.
            //Since these callbacks don't use per-body damping values, we can precalculate everything.
            linearDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt));
            angularDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt));
            gravityWideDt = Vector3Wide.Broadcast(gravity.Read<Vector3>() * dt);
        }

        /// <summary>
        /// Callback for a bundle of bodies being integrated.
        /// </summary>
        /// <param name="bodyIndices">Indices of the bodies being integrated in this bundle.</param>
        /// <param name="position">Current body positions.</param>
        /// <param name="orientation">Current body orientations.</param>
        /// <param name="localInertia">Body's current local inertia.</param>
        /// <param name="integrationMask">Mask indicating which lanes are active in the bundle. Active lanes will contain 0xFFFFFFFF, inactive lanes will contain 0.</param>
        /// <param name="workerIndex">Index of the worker thread processing this bundle.</param>
        /// <param name="dt">Durations to integrate the velocity over. Can vary over lanes.</param>
        /// <param name="velocity">Velocity of bodies in the bundle. Any changes to lanes which are not active by the integrationMask will be discarded.</param>
        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            //This is a handy spot to implement things like position dependent gravity or per-body damping.
            //This implementation uses a single damping value for all bodies that allows it to be precomputed.
            //We don't have to check for kinematics; IntegrateVelocityForKinematics returns false, so we'll never see them in this callback.
            //Note that these are SIMD operations and "Wide" types. There are Vector<float>.Count lanes of execution being evaluated simultaneously.
            //The types are laid out in array-of-structures-of-arrays (AOSOA) format. That's because this function is frequently called from vectorized contexts within the solver.
            //Transforming to "array of structures" (AOS) format for the callback and then back to AOSOA would involve a lot of overhead, so instead the callback works on the AOSOA representation directly.
            velocity.Linear = (velocity.Linear + gravityWideDt) * linearDampingDt;
            velocity.Angular = velocity.Angular * angularDampingDt;
        }
    }

    public class PhysicsSystem : SystemBase
    {
        private readonly Query<IsBody> bodyQuery;
        private readonly Query<IsBody, LinearVelocity> bodyLinearVelocityQuery;
        private readonly Query<IsGravitySource, DirectionalGravity> directionalGravityQuery;
        private readonly UnmanagedDictionary<eint, CompiledBody> bodies;
        private readonly UnmanagedDictionary<int, (TypedIndex, BodyInertia)> shapes;
        private readonly BepuPhysics.Simulation physicsSimulation;
        private readonly BufferPool bufferPool;
        private readonly Allocation gravity;

        public PhysicsSystem(World world) : base(world)
        {
            bufferPool = new();
            gravity = Allocation.Create(new Vector3(0f));
            DemoNarrowPhaseCallbacks narrowPhaseCallbacks = new(new SpringSettings(30, 1));
            DemoPoseIntegratorCallbacks poseIntegratorCallbacks = new(gravity, 0f, 0f);
            SolveDescription solveDescription = new(8, 1);
            physicsSimulation = BepuPhysics.Simulation.Create(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);

            bodyQuery = new(world);
            bodyLinearVelocityQuery = new(world);
            directionalGravityQuery = new(world);
            bodies = new();
            shapes = new();
            Subscribe<PhysicsUpdate>(Update);
        }

        public override void Dispose()
        {
            foreach (eint bodyEntity in bodies.Keys)
            {
                bodies[bodyEntity].Dispose();
            }

            shapes.Dispose();
            bodies.Dispose();
            directionalGravityQuery.Dispose();
            bodyLinearVelocityQuery.Dispose();
            bodyQuery.Dispose();
            physicsSimulation.Dispose();
            bufferPool.Clear();
            gravity.Dispose();
            base.Dispose();
        }

        private void Update(PhysicsUpdate update)
        {
            TimeSpan delta = update.delta;
            if (delta.TotalSeconds <= 0)
            {
                return;
            }

            Vector3 gravity = GetGlobalGravity();
            this.gravity.Write(0, gravity);
            uint created = UpdateBodiesToMatchEntities(gravity);
            float frequency = 1 / 60f;
            int steps = (int)MathF.Ceiling((float)delta.TotalSeconds / frequency);
            for (int i = 0; i < steps; i++)
            {
                //physicsSimulation.Timestep(frequency);
            }

            physicsSimulation.Timestep((float)delta.TotalSeconds);
            UpdateEntitiesToMatchBodies();
        }

        private Vector3 GetGlobalGravity()
        {
            Vector3 force = default;
            directionalGravityQuery.Update();
            foreach (var x in directionalGravityQuery)
            {
                LocalToWorld ltw = world.GetComponent(x.entity, LocalToWorld.Default);
                Quaternion rotation = ltw.Rotation;
                Vector3 forward = Vector3.Transform(Vector3.UnitZ, rotation);
                force += forward * x.Component1.force;
            }

            return force;
        }

        private uint UpdateBodiesToMatchEntities(Vector3 gravity)
        {
            //destroy bodies that dont point to their entity anymore
            using UnmanagedList<eint> toRemove = new();
            foreach (eint bodyEntity in bodies.Keys)
            {
                if (!world.ContainsEntity(bodyEntity))
                {
                    toRemove.Add(bodyEntity);
                }
            }

            foreach (eint bodyEntity in toRemove)
            {
                CompiledBody body = bodies.Remove(bodyEntity);
                body.Dispose();
            }

            uint addedCount = 0;
            bodyQuery.Update();
            foreach (var r in bodyQuery)
            {
                eint bodyEntity = r.entity;
                IsBody component = r.Component1;
                uint version = component.version;
                rint shapeReference = component.shapeReference;
                IsBody.Type type = component.type;
                if (!bodies.TryGetValue(bodyEntity, out CompiledBody body))
                {
                    //create if needed
                    body = CreateBody(bodyEntity, version, shapeReference, type);
                    bodies.Add(bodyEntity, body);
                    addedCount++;
                }
                else if (body.version != version || body.type != type)
                {
                    //recreate if version is different
                    body.Dispose();
                    body = CreateBody(bodyEntity, version, shapeReference, type);
                    bodies[bodyEntity] = body;
                }

                //update when entity component data is different
                LocalToWorld ltw = world.GetComponent(bodyEntity, LocalToWorld.Default);
                float mass = world.GetComponent(bodyEntity, Mass.Default).value;
                Vector3 worldPosition = ltw.Position;
                Quaternion worldRotation = ltw.Rotation;
                if (world.TryGetComponent(bodyEntity, out Rotation rotationComponent))
                {
                    worldRotation = rotationComponent.value;
                }

                Vector3 scale = ltw.Scale;
                Vector3 offset = world.GetComponent<IsShape>(body.shapeEntity).offset;
                worldPosition += Vector3.Transform(offset, worldRotation);
                if (type == IsBody.Type.Dynamic || type == IsBody.Type.Kinematic)
                {
                    Vector3 linearVelocity = world.GetComponent(bodyEntity, new LinearVelocity()).value;
                    Vector3 angularVelocity = world.GetComponent(bodyEntity, new AngularVelocity()).value;
                    BodyHandle bodyHandle = body.DynamicBody;
                    BodyReference bodyReference = physicsSimulation.Bodies[bodyHandle];
                    if (worldPosition != body.lastPosition)
                    {
                        ref RigidPose pose = ref bodyReference.Pose;
                        pose.Position = worldPosition;
                        body.lastPosition = worldPosition;
                    }

                    if (worldRotation != body.lastRotation)
                    {
                        ref RigidPose pose = ref bodyReference.Pose;
                        pose.Orientation = worldRotation;
                        body.lastRotation = worldRotation;
                    }

                    if (linearVelocity != body.lastLinearVelocity)
                    {
                        ref BodyVelocity velocity = ref bodyReference.Velocity;
                        velocity.Linear = linearVelocity;
                        bodyReference.Awake = true;
                        body.lastLinearVelocity = linearVelocity;
                    }

                    if (angularVelocity != body.lastAngularVelocity)
                    {
                        ref BodyVelocity velocity = ref bodyReference.Velocity;
                        velocity.Angular = angularVelocity;
                        bodyReference.Awake = true;
                        body.lastAngularVelocity = angularVelocity;
                    }

                    if (mass != body.lastMass || scale != body.lastScale)
                    {
                        //shapes are dependent on the mass, so build a new shape
                        //todo: remove shapes that arent used? so the shape count doesnt bloat if mass updates at runtime?
                        (TypedIndex shapeIndex, BodyInertia bodyInertia) shape;
                        int shapeHash;
                        eint shapeEntity = world.GetReference(bodyEntity, shapeReference);
                        if (world.TryGetComponent(shapeEntity, out CubeShape cube))
                        {
                            Vector3 extents = cube.extents * scale;
                            shapeHash = HashCode.Combine(extents, mass, 0);
                            if (!shapes.TryGetValue(shapeHash, out shape))
                            {
                                Box boxShape = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(boxShape);
                                BodyInertia bodyInertia = boxShape.ComputeInertia(mass);
                                shape = (shapeIndex, bodyInertia);
                                shapes.Add(shapeHash, shape);
                            }
                        }
                        else if (world.TryGetComponent(shapeEntity, out SphereShape sphere))
                        {
                            float radius = sphere.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                            shapeHash = HashCode.Combine(radius, mass, 1);
                            if (!shapes.TryGetValue(shapeHash, out shape))
                            {
                                Sphere sphereShape = new(radius);
                                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(sphereShape);
                                BodyInertia bodyInertia = sphereShape.ComputeInertia(mass);
                                shape = (shapeIndex, bodyInertia);
                                shapes.Add(shapeHash, shape);
                            }
                        }
                        else
                        {
                            throw new Exception($"Shape entity `{shapeEntity}` does not have a shape component.");
                        }

                        if (scale != body.lastScale)
                        {
                            physicsSimulation.Bodies[bodyHandle].SetShape(shape.shapeIndex);
                        }

                        if (mass != body.lastMass)
                        {
                            physicsSimulation.Bodies[bodyHandle].SetLocalInertia(shape.bodyInertia);
                        }

                        body.shapeHash = shapeHash;
                        body.lastMass = mass;
                        body.lastScale = scale;
                    }
                }
                else if (type == IsBody.Type.Static)
                {
                    StaticHandle staticHandle = body.StaticBody;
                    StaticReference staticReference = physicsSimulation.Statics[staticHandle];
                    if (worldPosition != body.lastPosition)
                    {
                        StaticDescription description = staticReference.GetDescription();
                        description.Pose.Position = worldPosition;
                        body.lastPosition = worldPosition;
                        physicsSimulation.Statics.ApplyDescription(staticHandle, description);
                    }

                    if (worldRotation != body.lastRotation)
                    {
                        StaticDescription description = staticReference.GetDescription();
                        description.Pose.Orientation = worldRotation;
                        body.lastRotation = worldRotation;
                        physicsSimulation.Statics.ApplyDescription(staticHandle, description);
                    }

                    if (scale != body.lastScale)
                    {
                        //size changed, shape needs to be rebuilt
                        (TypedIndex shapeIndex, BodyInertia bodyInertia) shape;
                        int shapeHash;
                        eint shapeEntity = world.GetReference(bodyEntity, shapeReference);
                        if (world.TryGetComponent(shapeEntity, out CubeShape cube))
                        {
                            Vector3 extents = cube.extents * scale;
                            shapeHash = HashCode.Combine(extents, mass, 0);
                            if (!shapes.TryGetValue(shapeHash, out shape))
                            {
                                Box boxShape = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(boxShape);
                                BodyInertia bodyInertia = boxShape.ComputeInertia(mass);
                                shape = (shapeIndex, bodyInertia);
                                shapes.Add(shapeHash, shape);
                            }
                        }
                        else if (world.TryGetComponent(shapeEntity, out SphereShape sphere))
                        {
                            float radius = sphere.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                            shapeHash = HashCode.Combine(radius, mass, 1);
                            if (!shapes.TryGetValue(shapeHash, out shape))
                            {
                                Sphere sphereShape = new(radius);
                                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(sphereShape);
                                BodyInertia bodyInertia = sphereShape.ComputeInertia(mass);
                                shape = (shapeIndex, bodyInertia);
                                shapes.Add(shapeHash, shape);
                            }
                        }
                        else
                        {
                            throw new Exception($"Shape entity `{shapeEntity}` does not have a shape component.");
                        }

                        StaticDescription newDescription = staticReference.GetDescription();
                        newDescription.Shape = shape.shapeIndex;
                        staticReference.ApplyDescription(newDescription);
                        body.shapeHash = shapeHash;
                        body.lastMass = mass;
                        body.lastScale = scale;
                    }
                }

                bodies[bodyEntity] = body;
            }

            return addedCount + toRemove.Count;
        }

        private void UpdateEntitiesToMatchBodies()
        {
            foreach (eint bodyEntity in bodies.Keys)
            {
                if (world.TryGetComponent(bodyEntity, out LocalToWorld ltw))
                {
                    ref CompiledBody body = ref bodies[bodyEntity];
                    Matrix4x4.Invert(ltw.value, out Matrix4x4 wtl);
                    if (world.GetParent(bodyEntity) == default)
                    {
                        wtl = Matrix4x4.Identity;
                    }

                    Vector3 offset = world.GetComponent<IsShape>(body.shapeEntity).offset;
                    if (body.type == IsBody.Type.Dynamic || body.type == IsBody.Type.Kinematic)
                    {
                        BodyReference physicsBody = physicsSimulation.Bodies[body.DynamicBody];
                        Vector3 worldPosition = physicsBody.Pose.Position - offset;
                        Quaternion worldRotation = physicsBody.Pose.Orientation;
                        Vector3 newLocalPosition = Vector3.Transform(worldPosition, wtl);
                        Quaternion newLocalRotation = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(wtl) * worldRotation);
                        ref Position positionComponent = ref world.TryGetComponentRef<Position>(bodyEntity, out bool contains);
                        if (contains)
                        {
                            positionComponent.value = newLocalPosition;
                        }
                        else
                        {
                            world.AddComponent(bodyEntity, new Position(newLocalPosition));
                        }

                        ref Rotation rotationComponent = ref world.TryGetComponentRef<Rotation>(bodyEntity, out contains);
                        if (contains)
                        {
                            rotationComponent.value = newLocalRotation;
                        }
                        else
                        {
                            world.AddComponent(bodyEntity, new Rotation(newLocalRotation));
                        }

                        Vector3 linearVelocity = physicsBody.Velocity.Linear;
                        ref LinearVelocity linearVelocityComponent = ref world.TryGetComponentRef<LinearVelocity>(bodyEntity, out contains);
                        if (contains)
                        {
                            linearVelocityComponent.value = linearVelocity;
                        }
                        else
                        {
                            world.AddComponent(bodyEntity, new LinearVelocity(linearVelocity));
                        }

                        Vector3 angularVelocity = physicsBody.Velocity.Angular;
                        ref AngularVelocity angularVelocityComponent = ref world.TryGetComponentRef<AngularVelocity>(bodyEntity, out contains);
                        if (contains)
                        {
                            angularVelocityComponent.value = angularVelocity;
                        }
                        else
                        {
                            world.AddComponent(bodyEntity, new AngularVelocity(angularVelocity));
                        }

                        body.lastPosition = worldPosition;
                        body.lastRotation = worldRotation;
                        body.lastLinearVelocity = linearVelocity;
                        body.lastAngularVelocity = angularVelocity;

                        //update boundaries
                        Vector3 min = physicsBody.BoundingBox.Min;
                        Vector3 max = physicsBody.BoundingBox.Max;
                        ref WorldBounds boundsComponent = ref world.TryGetComponentRef<WorldBounds>(bodyEntity, out contains);
                        if (contains)
                        {
                            boundsComponent.min = min;
                            boundsComponent.max = max;
                        }
                        else
                        {
                            world.AddComponent(bodyEntity, new WorldBounds(min, max));
                        }
                    }
                    else if (body.type == IsBody.Type.Static)
                    {
                        //why would a static body be different after a step?
                    }
                }
            }
        }

        private CompiledBody CreateBody(eint bodyEntity, uint version, rint shapeReference, IsBody.Type type)
        {
            eint shapeEntity = world.GetReference(bodyEntity, shapeReference);
            IsShape component = world.GetComponent<IsShape>(shapeEntity);
            Vector3 offset = component.offset;
            LocalToWorld ltw = world.GetComponent(bodyEntity, LocalToWorld.Default);
            float mass = world.GetComponent(bodyEntity, Mass.Default).value;
            Vector3 linearVelocity = world.GetComponent(bodyEntity, new LinearVelocity()).value;
            Vector3 position = ltw.Position;
            Quaternion rotation = ltw.Rotation;
            if (world.TryGetComponent(bodyEntity, out Rotation rotationComponent))
            {
                rotation = rotationComponent.value;
            }

            Vector3 scale = ltw.Scale;
            position += Vector3.Transform(offset, rotation);
            int shapeHash;
            (TypedIndex shapeIndex, BodyInertia bodyInertia) shape;
            if (world.TryGetComponent(shapeEntity, out CubeShape cube))
            {
                Vector3 extents = cube.extents * scale;
                shapeHash = HashCode.Combine(extents, mass, 0);
                if (!shapes.TryGetValue(shapeHash, out shape))
                {
                    Box boxShape = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                    TypedIndex shapeIndex = physicsSimulation.Shapes.Add(boxShape);
                    BodyInertia bodyInertia = boxShape.ComputeInertia(mass);
                    shape = (shapeIndex, bodyInertia);
                    shapes.Add(shapeHash, shape);
                }
            }
            else if (world.TryGetComponent(shapeEntity, out SphereShape sphere))
            {
                float radius = sphere.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                shapeHash = HashCode.Combine(radius, mass, 1);
                if (!shapes.TryGetValue(shapeHash, out shape))
                {
                    Sphere sphereShape = new(radius);
                    TypedIndex shapeIndex = physicsSimulation.Shapes.Add(sphereShape);
                    BodyInertia bodyInertia = sphereShape.ComputeInertia(mass);
                    shape = (shapeIndex, bodyInertia);
                    shapes.Add(shapeHash, shape);
                }
            }
            else
            {
                throw new Exception($"Shape entity `{shapeEntity}` does not have a shape component.");
            }

            RigidPose pose = new(position, rotation);
            ContinuousDetection continuity = ContinuousDetection.Discrete;
            BodyActivityDescription activity = new(0.01f);
            int intHandle = 0;
            if (type == IsBody.Type.Dynamic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateDynamic(pose, linearVelocity, shape.bodyInertia, collidable, activity);
                BodyHandle handle = physicsSimulation.Bodies.Add(description);
                intHandle = handle.Value;
            }
            else if (type == IsBody.Type.Kinematic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateKinematic(pose, linearVelocity, collidable, activity);
                BodyHandle handle = physicsSimulation.Bodies.Add(description);
                intHandle = handle.Value;
            }
            else if (type == IsBody.Type.Static)
            {
                StaticDescription description = new(pose, shape.shapeIndex, continuity);
                StaticHandle handle = physicsSimulation.Statics.Add(description);
                intHandle = handle.Value;
            }

            CompiledBody body = new(version, intHandle, shapeHash, shapeEntity, type);
            body.lastMass = mass;
            body.lastLinearVelocity = linearVelocity;
            body.lastScale = scale;
            return body;
        }
    }
}