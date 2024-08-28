using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
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
    public class PhysicsSystem : SystemBase
    {
        private readonly Query<IsShape> shapeQuery;
        private readonly Query<IsRaycaster, LocalToWorld, WorldRotation> raycasterQuery;
        private readonly Query<IsBody, LocalToWorld, WorldRotation> bodyQuery;
        private readonly Query<IsBody, LinearVelocity> bodyLinearVelocityQuery;
        private readonly Query<IsGravitySource, IsDirectionalGravity> directionalGravityQuery;
        private readonly UnmanagedArray<(Vector3, Quaternion, Vector3, Vector3)> physicsObjectState;
        private readonly UnmanagedDictionary<eint, CompiledBody> bodies;
        private readonly UnmanagedDictionary<int, CompiledShape> shapes;
        private readonly UnmanagedDictionary<(int, bool), eint> handleToBody;
        private readonly BepuPhysics.Simulation physicsSimulation;
        private readonly BufferPool bufferPool;
        private readonly Allocation gravity;

        public PhysicsSystem(World world) : base(world)
        {
            bufferPool = new();
            gravity = Allocation.Create(new Vector3(0f));
            NarrowPhaseCallbacks narrowPhaseCallbacks = new(new SpringSettings(30, 1));
            PoseIntegratorCallbacks poseIntegratorCallbacks = new(gravity, 0f, 0f);
            SolveDescription solveDescription = new(8, 1);
            physicsSimulation = BepuPhysics.Simulation.Create(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);

            shapeQuery = new(world);
            raycasterQuery = new(world);
            bodyQuery = new(world);
            bodyLinearVelocityQuery = new(world);
            directionalGravityQuery = new(world);
            physicsObjectState = new();
            bodies = new();
            shapes = new();
            handleToBody = new();
            Subscribe<PhysicsUpdate>(Update);
        }

        public override void Dispose()
        {
            foreach (eint bodyEntity in bodies.Keys)
            {
                bodies[bodyEntity].Dispose();
            }

            foreach (int shapeHash in shapes.Keys)
            {
                shapes[shapeHash].Dispose();
            }

            handleToBody.Dispose();
            shapes.Dispose();
            bodies.Dispose();
            directionalGravityQuery.Dispose();
            bodyLinearVelocityQuery.Dispose();
            physicsObjectState.Dispose();
            bodyQuery.Dispose();
            physicsSimulation.Dispose();
            bufferPool.Clear();
            gravity.Dispose();
            raycasterQuery.Dispose();
            shapeQuery.Dispose();
            base.Dispose();
        }

        private void Update(PhysicsUpdate update)
        {
            TimeSpan delta = update.delta;
            if (delta.TotalSeconds <= 0)
            {
                return;
            }

            gravity.Write(0, GetGlobalGravity());
            CreateAndDestroyPhysicsObjects();
            physicsSimulation.Timestep((float)delta.TotalSeconds);
            CopyPhysicsObjectStateToEntities();
            PerformRaycasts(delta);
        }

        private void CreateAndDestroyPhysicsObjects()
        {
            //remove shapes and bodies that arent pointing to entities anymore
            using UnmanagedList<eint> bodiesRemoved = new();
            foreach (eint bodyEntity in bodies.Keys)
            {
                if (!world.ContainsEntity(bodyEntity))
                {
                    CompiledBody body = bodies[bodyEntity];
                    if (body.type == IsBody.Type.Dynamic || body.type == IsBody.Type.Kinematic)
                    {
                        handleToBody.Remove((body.DynamicBody.Value, false));
                        physicsSimulation.Bodies.Remove(body.DynamicBody);
                    }
                    else if (body.type == IsBody.Type.Static)
                    {
                        handleToBody.Remove((body.StaticBody.Value, true));
                        physicsSimulation.Statics.Remove(body.StaticBody);
                    }

                    physicsObjectState[(uint)bodyEntity] = default;
                    body.Dispose();
                    bodiesRemoved.Add(bodyEntity);
                }
            }

            foreach (eint bodyEntity in bodiesRemoved)
            {
                bodies.Remove(bodyEntity);
            }

            physicsObjectState.Resize(world.MaxEntityValue + 1);
            bodyQuery.Update();
            using UnmanagedList<int> usedShapes = new();
            foreach (var r in bodyQuery)
            {
                eint bodyEntity = r.entity;
                IsBody bodyComponent = r.Component1;
                LocalToWorld ltw = r.Component2;
                eint shapeEntity = world.GetReference(bodyEntity, bodyComponent.shapeReference);
                if (world.TryGetComponent(shapeEntity, out IsShape shapeComponent))
                {
                    //make sure a reusable shape exists for this combination of (type, offset, mass, scale)
                    IsBody.Type type = bodyComponent.type;
                    bool newShape = false;
                    float mass;
                    if (type == IsBody.Type.Static)
                    {
                        mass = float.MaxValue;
                    }
                    else if (type == IsBody.Type.Dynamic)
                    {
                        mass = world.GetComponent<Mass>(bodyEntity).value;
                    }
                    else if (type == IsBody.Type.Kinematic)
                    {
                        mass = float.MaxValue;
                    }
                    else
                    {
                        throw new Exception($"Physics body `{bodyEntity}` has an unrecognized body type `{type}`");
                    }

                    Vector3 scale = ltw.Scale;
                    Vector3 roundedScale = new(MathF.Round(scale.X, 3), MathF.Round(scale.Y, 3), MathF.Round(scale.Z, 3));
                    Vector3 localOffset = shapeComponent.offset;
                    int shapeHash = HashCode.Combine(shapeEntity, type, localOffset, roundedScale, mass);
                    if (!shapes.TryGetValue(shapeHash, out CompiledShape compiledShape))
                    {
                        compiledShape = CreateShape(shapeEntity, localOffset, scale, mass);
                        shapes.Add(shapeHash, compiledShape);
                        newShape = true;
                    }

                    usedShapes.Add(shapeHash);

                    //make sure a physics body exists for this combination of (shape, type)
                    Vector3 worldOffset = Vector3.Transform(localOffset, ltw.Rotation);
                    //worldOffset = default;
                    Vector3 desiredWorldPosition = ltw.Position + worldOffset;
                    Quaternion desiredWorldRotation = r.Component3.value;
                    if (!bodies.TryGetValue(bodyEntity, out CompiledBody compiledBody))
                    {
                        compiledBody = CreateBody(bodyComponent, compiledShape, desiredWorldPosition, desiredWorldRotation);
                        bodies.Add(bodyEntity, compiledBody);

                        if (type == IsBody.Type.Static)
                        {
                            handleToBody.Add((compiledBody.StaticBody.Value, true), bodyEntity);
                        }
                        else
                        {
                            handleToBody.Add((compiledBody.DynamicBody.Value, false), bodyEntity);
                        }
                    }
                    else if (compiledBody.version != bodyComponent.version || compiledBody.type != type || newShape)
                    {
                        if (compiledBody.type == IsBody.Type.Static)
                        {
                            handleToBody.Remove((compiledBody.StaticBody.Value, true));
                            physicsSimulation.Statics.Remove(compiledBody.StaticBody);
                        }
                        else
                        {
                            handleToBody.Remove((compiledBody.DynamicBody.Value, false));
                            physicsSimulation.Bodies.Remove(compiledBody.DynamicBody);
                        }

                        compiledBody.Dispose();
                        compiledBody = CreateBody(bodyComponent, compiledShape, desiredWorldPosition, desiredWorldRotation);
                        bodies[bodyEntity] = compiledBody;

                        if (type == IsBody.Type.Static)
                        {
                            handleToBody.Add((compiledBody.StaticBody.Value, true), bodyEntity);
                        }
                        else
                        {
                            handleToBody.Add((compiledBody.DynamicBody.Value, false), bodyEntity);
                        }
                    }

                    //copy values from entity onto physics object (if different from last known state)
                    if (type == IsBody.Type.Dynamic || type == IsBody.Type.Kinematic)
                    {
                        (Vector3 position, Quaternion rotation, Vector3 linear, Vector3 angular) = physicsObjectState[(uint)bodyEntity];
                        BodyReference bodyReference = physicsSimulation.Bodies[compiledBody.DynamicBody];
                        if (position != desiredWorldPosition || rotation != desiredWorldRotation)
                        {
                            ref RigidPose pose = ref bodyReference.Pose;
                            pose.Position = desiredWorldPosition;
                            pose.Orientation = desiredWorldRotation;
                            bodyReference.Awake = true;
                        }

                        Vector3 linearVelocity = world.GetComponent(bodyEntity, new LinearVelocity()).value; //optional
                        Vector3 angularVelocity = world.GetComponent(bodyEntity, new AngularVelocity()).value; //optional
                        if (linear != linearVelocity || angular != angularVelocity)
                        {
                            ref BodyVelocity velocity = ref bodyReference.Velocity;
                            velocity.Linear = linearVelocity;
                            velocity.Angular = angularVelocity;
                            bodyReference.Awake = true;
                        }
                    }
                    else if (type == IsBody.Type.Static)
                    {
                        (Vector3 position, Quaternion rotation, Vector3 linear, Vector3 angular) state = physicsObjectState[(uint)bodyEntity];
                        StaticReference staticReference = physicsSimulation.Statics[compiledBody.StaticBody];
                        if (state.position != desiredWorldPosition || state.rotation != desiredWorldRotation)
                        {
                            state.position = desiredWorldPosition;
                            state.rotation = desiredWorldRotation;
                            StaticDescription newDescription = staticReference.GetDescription();
                            newDescription.Pose.Position = desiredWorldPosition;
                            newDescription.Pose.Orientation = desiredWorldRotation;
                            staticReference.ApplyDescription(newDescription);
                            physicsObjectState[(uint)bodyEntity] = state;
                        }
                    }
                }
                else
                {
                    throw new Exception($"Shape `{shapeEntity}` of physics body `{bodyEntity}` doesn't contain shape information");
                }
            }

            //remove shapes that are no longer used
            using UnmanagedList<int> shapesToRemove = new();
            foreach (int shapeHash in shapes.Keys)
            {
                if (!usedShapes.Contains(shapeHash))
                {
                    shapesToRemove.Add(shapeHash);
                }
            }

            foreach (int shapeHash in shapesToRemove)
            {
                CompiledShape shape = shapes.Remove(shapeHash);
                physicsSimulation.Shapes.Remove(shape.shapeIndex);
                shape.Dispose();
            }
        }

        private void CopyPhysicsObjectStateToEntities()
        {
            foreach (eint bodyEntity in bodies.Keys)
            {
                CompiledBody body = bodies[bodyEntity];
                rint shapeReference = world.GetComponent<IsBody>(bodyEntity).shapeReference;
                eint shapeEntity = world.GetReference(bodyEntity, shapeReference);
                if (!world.TryGetComponent(shapeEntity, out IsShape shapeComponent))
                {
                    continue;
                }

                LocalToWorld ltw = world.GetComponent<LocalToWorld>(bodyEntity);
                Vector3 localOffset = shapeComponent.offset;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltw.Rotation);
                //worldOffset = default;
                if (body.type == IsBody.Type.Dynamic || body.type == IsBody.Type.Kinematic)
                {
                    eint bodyParent = world.GetParent(bodyEntity);
                    Matrix4x4 wtl = Matrix4x4.Identity;
                    if (bodyParent != default)
                    {
                        Matrix4x4.Invert(world.GetComponent(bodyParent, LocalToWorld.Default).value, out wtl);
                    }

                    //copy into individual local components
                    BodyReference bodyReference = physicsSimulation.Bodies[body.DynamicBody];
                    RigidPose pose = bodyReference.Pose;
                    if (!world.ContainsComponent<Position>(bodyEntity))
                    {
                        world.AddComponent<Position>(bodyEntity);
                    }

                    if (!world.ContainsComponent<Rotation>(bodyEntity))
                    {
                        world.AddComponent<Rotation>(bodyEntity);
                    }

                    if (!world.ContainsComponent<LinearVelocity>(bodyEntity))
                    {
                        world.AddComponent<LinearVelocity>(bodyEntity);
                    }

                    if (!world.ContainsComponent<AngularVelocity>(bodyEntity))
                    {
                        world.AddComponent<AngularVelocity>(bodyEntity);
                    }

                    ref Position localPosition = ref world.GetComponentRef<Position>(bodyEntity);
                    Vector3 finalWorldPosition = pose.Position - worldOffset;
                    localPosition.value = Vector3.Transform(finalWorldPosition, wtl);

                    ref Rotation localRotation = ref world.GetComponentRef<Rotation>(bodyEntity);
                    Quaternion finalWorldRotation = pose.Orientation;
                    if (bodyParent != default)
                    {
                        localRotation.value = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(wtl) * finalWorldRotation);
                    }
                    else
                    {
                        localRotation.value = finalWorldRotation;
                    }

                    ref LinearVelocity linearVelocity = ref world.GetComponentRef<LinearVelocity>(bodyEntity);
                    BodyVelocity velocity = bodyReference.Velocity;
                    linearVelocity.value = velocity.Linear;

                    ref AngularVelocity angularVelocity = ref world.GetComponentRef<AngularVelocity>(bodyEntity);
                    angularVelocity.value = velocity.Angular;
                    physicsObjectState[(uint)bodyEntity] = (finalWorldPosition, finalWorldRotation, velocity.Linear, velocity.Angular);

                    //copy bounds
                    if (!world.ContainsComponent<WorldBounds>(bodyEntity))
                    {
                        world.AddComponent(bodyEntity, new WorldBounds(bodyReference.BoundingBox.Min, bodyReference.BoundingBox.Max));
                    }
                    else
                    {
                        ref WorldBounds bounds = ref world.GetComponentRef<WorldBounds>(bodyEntity);
                        bounds.min = bodyReference.BoundingBox.Min;
                        bounds.max = bodyReference.BoundingBox.Max;
                    }
                }
                else if (body.type == IsBody.Type.Static)
                {
                    StaticReference staticReference = physicsSimulation.Statics[body.StaticBody];
                    StaticDescription description = staticReference.GetDescription();
                    Vector3 finalWorldPosition = description.Pose.Position - worldOffset;
                    Quaternion finalWorldRotation = description.Pose.Orientation;
                    physicsObjectState[(uint)bodyEntity] = (finalWorldPosition, finalWorldRotation, Vector3.Zero, Vector3.Zero);

                    //copy bounds
                    if (!world.ContainsComponent<WorldBounds>(bodyEntity))
                    {
                        world.AddComponent(bodyEntity, new WorldBounds(staticReference.BoundingBox.Min, staticReference.BoundingBox.Max));
                    }
                    else
                    {
                        ref WorldBounds bounds = ref world.GetComponentRef<WorldBounds>(bodyEntity);
                        bounds.min = staticReference.BoundingBox.Min;
                        bounds.max = staticReference.BoundingBox.Max;
                    }
                }
            }
        }

        private void PerformRaycasts(TimeSpan delta)
        {
            using UnmanagedList<(bool, int, float, Vector3)> hits = new();
            Span<RaycastHit> raycastHits = stackalloc RaycastHit[32];
            raycasterQuery.Update();
            foreach (var r in raycasterQuery)
            {
                eint raycastEntity = r.entity;
                if (!world.IsEnabled(raycastEntity)) continue;

                IsRaycaster component = r.Component1;
                LocalToWorld ltw = r.Component2;
                Vector3 worldPosition = ltw.Position;
                Quaternion worldRotation = r.Component3.value;
                Vector3 worldForward = Vector3.Transform(Vector3.UnitZ, worldRotation);

                RaycastHandler handler = new(hits);
                physicsSimulation.RayCast(worldPosition, worldForward, component.maxDistance, ref handler);
                int hitCount = 0;
                for (uint i = 0; i < hits.Count; i++)
                {
                    (bool isStatic, int handle, float distance, Vector3 normal) = hits[i];
                    if (handleToBody.TryGetValue((handle, isStatic), out eint hitEntity))
                    {
                        Vector3 point = worldPosition + worldForward * distance;
                        RaycastHit hit = new(point, normal, distance, hitEntity);
                        raycastHits[hitCount] = hit;
                        hitCount++;
                        if (component.callback != default)
                        {
                            component.callback.Invoke(world, raycastEntity, hit, delta);
                        }
                    }
                }

                if (!world.ContainsArray<RaycastHit>(raycastEntity))
                {
                    world.CreateArray<RaycastHit>(raycastEntity, raycastHits[..hitCount]);
                }
                else
                {
                    Span<RaycastHit> resultsArray = world.ResizeArray<RaycastHit>(raycastEntity, hits.Count);
                    raycastHits[..hitCount].CopyTo(resultsArray);
                }
            }

        }

        private CompiledShape CreateShape(eint shapeEntity, Vector3 offset, Vector3 scale, float mass)
        {
            if (world.TryGetComponent(shapeEntity, out IsCubeShape cubeShape))
            {
                Vector3 extents = cubeShape.extents * scale;
                Box box = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(box);
                BodyInertia bodyInertia = box.ComputeInertia(mass);
                return new CompiledShape(offset, scale, shapeIndex, bodyInertia);
            }
            else if (world.TryGetComponent(shapeEntity, out IsSphereShape sphereShape))
            {
                float radius = sphereShape.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                Sphere sphere = new(radius);
                TypedIndex shapeIndex = physicsSimulation.Shapes.Add(sphere);
                BodyInertia bodyInertia = sphere.ComputeInertia(mass);
                return new CompiledShape(offset, scale, shapeIndex, bodyInertia);
            }
            else
            {
                throw new Exception($"Shape entity `{shapeEntity}` is not recognized as a valid shape");
            }
        }

        private CompiledBody CreateBody(IsBody bodyComponent, CompiledShape shape, Vector3 worldPosition, Quaternion worldRotation)
        {
            IsBody.Type type = bodyComponent.type;
            RigidPose pose = new(worldPosition, worldRotation);
            ContinuousDetection continuity = ContinuousDetection.Discrete;
            BodyActivityDescription activity = new(0.01f);
            uint version = bodyComponent.version;
            int intHandle = 0;
            if (type == IsBody.Type.Dynamic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateDynamic(pose, default, shape.bodyInertia, collidable, activity);
                BodyHandle handle = physicsSimulation.Bodies.Add(description);
                intHandle = handle.Value;
            }
            else if (type == IsBody.Type.Kinematic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateKinematic(pose, default, collidable, activity);
                BodyHandle handle = physicsSimulation.Bodies.Add(description);
                intHandle = handle.Value;
            }
            else if (type == IsBody.Type.Static)
            {
                StaticDescription description = new(pose, shape.shapeIndex, continuity);
                StaticHandle handle = physicsSimulation.Statics.Add(description);
                intHandle = handle.Value;
            }

            return new(version, intHandle, type);
        }

        private Vector3 GetGlobalGravity()
        {
            Vector3 force = default;
            directionalGravityQuery.Update();
            foreach (var x in directionalGravityQuery)
            {
                LocalToWorld ltw = world.GetComponent<LocalToWorld>(x.entity);
                Quaternion rotation = ltw.Rotation;
                Vector3 forward = Vector3.Transform(Vector3.UnitZ, rotation);
                force += forward * x.Component1.force;
            }

            return force;
        }
    }
}