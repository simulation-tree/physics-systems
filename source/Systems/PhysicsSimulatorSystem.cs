using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using Collections;
using Physics.Components;
using Physics.Events;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Worlds;

namespace Physics.Systems
{
    public readonly struct PhysicsSimulatorSystem : IDisposable
    {
        private readonly List<RaycastHit> hits;
        private readonly List<uint> toRemove;
        private readonly List<uint> usedShapes;
        private readonly Array<BodyState> physicsObjectState;
        private readonly Dictionary<uint, CompiledBody> bodies;
        private readonly Dictionary<uint, CompiledShape> shapes;
        private readonly Dictionary<(int, bool), uint> handleToBody;
        private readonly List<(Vector3, float, float)> pointGravitySources;
        private readonly BepuSimulation bepuSimulation;
        private readonly BepuBufferPool bufferPool;
        private readonly Allocation gravity;
        private readonly World world;
        private readonly Operation operation;

        public PhysicsSimulatorSystem(World world)
        {
            this.world = world;
            bufferPool = new();
            gravity = Allocation.Create(new Vector3(0f));
            NarrowPhaseCallbacks narrowPhaseCallbacks = new(new SpringSettings(30, 1));
            PoseIntegratorCallbacks poseIntegratorCallbacks = new(gravity, 0f, 0f);
            SolveDescription solveDescription = new(8, 1);
            bepuSimulation = new(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);

            hits = new();
            toRemove = new();
            usedShapes = new();
            physicsObjectState = new();
            bodies = new();
            shapes = new();
            handleToBody = new();
            pointGravitySources = new();
            operation = new();
        }

        public readonly void Dispose()
        {
            operation.Dispose();

            foreach (uint bodyEntity in bodies.Keys)
            {
                bodies[bodyEntity].Dispose();
            }

            foreach (uint shapeHash in shapes.Keys)
            {
                shapes[shapeHash].Dispose();
            }

            pointGravitySources.Dispose();
            handleToBody.Dispose();
            shapes.Dispose();
            bodies.Dispose();
            physicsObjectState.Dispose();
            bepuSimulation.Dispose();
            bufferPool.Dispose();
            gravity.Dispose();
            usedShapes.Dispose();
            toRemove.Dispose();
            hits.Dispose();
        }

        public readonly void Update(TimeSpan delta)
        {
            gravity.Write(GetGlobalGravity());
            AddMissingComponents();
            ApplyPointGravity((float)delta.TotalSeconds);
            CreateAndDestroyPhysicsObjects();

            if (delta.Ticks > 0)
            {
                bepuSimulation.Update(delta);
            }

            CopyPhysicsObjectStateToEntities();
        }

        private readonly void ApplyPointGravity(float delta)
        {
            FindPointGravitySources();

            ComponentQuery<IsBody, LocalToWorld, LinearVelocity> bodyQuery = new(world);
            foreach (var r in bodyQuery)
            {
                ref IsBody body = ref r.component1;
                ref LocalToWorld ltw = ref r.component2;
                ref LinearVelocity linearVelocity = ref r.component3;
                if (body.type == BodyType.Dynamic)
                {
                    Vector3 accumulatedGravity = default;
                    Vector3 worldPosition = ltw.Position;
                    foreach ((Vector3 sourcePosition, float force, float radius) in pointGravitySources)
                    {
                        Vector3 offset = worldPosition - sourcePosition;
                        float distanceSquared = offset.LengthSquared();
                        if (distanceSquared < radius * radius)
                        {
                            float distance = MathF.Sqrt(distanceSquared);
                            float attenuation = 1f - MathF.Min(distance / radius, 1f);
                            Vector3 direction = Vector3.Normalize(offset);
                            accumulatedGravity += direction * force * attenuation;
                        }
                    }

                    if (accumulatedGravity != default)
                    {
                        accumulatedGravity *= -1;
                        linearVelocity.value += accumulatedGravity * delta;
                    }
                }
            }
        }

        private readonly void FindPointGravitySources()
        {
            pointGravitySources.Clear();
            ComponentQuery<IsGravitySource, IsPointGravity, LocalToWorld> pointGravityQuery = new(world);
            foreach (var r in pointGravityQuery)
            {
                ref IsGravitySource gravitySource = ref r.component1;
                ref IsPointGravity pointGravity = ref r.component2;
                ref LocalToWorld ltw = ref r.component3;
                pointGravitySources.Add((ltw.Position, gravitySource.force, pointGravity.radius));
            }
        }

        private readonly void AddMissingComponents()
        {
            Schema schema = world.Schema;

            //make sure linear velocity exists
            ComponentQuery<IsBody> bodiesWithoutVelocityQuery = new(world);
            bodiesWithoutVelocityQuery.ExcludeComponent<LinearVelocity>();
            bool changed = false;
            foreach (var r in bodiesWithoutVelocityQuery)
            {
                ref IsBody body = ref r.component1;
                if (body.type == BodyType.Dynamic)
                {
                    operation.SelectEntity(r.entity);
                    changed = true;
                }
            }

            if (changed)
            {
                operation.AddComponent<LinearVelocity>(schema);
                operation.ClearSelection();
                changed = false;
            }

            //make sure angular velocity is present
            ComponentQuery<IsBody> bodeiesWithoutAngularVelocity = new(world);
            bodeiesWithoutAngularVelocity.ExcludeComponent<AngularVelocity>();
            foreach (var r in bodeiesWithoutAngularVelocity)
            {
                ref IsBody body = ref r.component1;
                if (body.type == BodyType.Dynamic)
                {
                    operation.SelectEntity(r.entity);
                    changed = true;
                }
            }

            if (changed)
            {
                operation.AddComponent<AngularVelocity>(schema);
                operation.ClearSelection();
                changed = false;
            }

            //make sure position exists
            ComponentQuery<IsBody, LocalToWorld> positionMissingQuery = new(world);
            positionMissingQuery.ExcludeComponent<Position>();
            foreach (var r in positionMissingQuery)
            {
                ref IsBody body = ref r.component1;
                if (body.type != BodyType.Static)
                {
                    operation.SelectEntity(r.entity);
                    changed = true;
                }
            }

            if (changed)
            {
                operation.AddComponent<Position>(schema);
                operation.ClearSelection();
                changed = false;
            }

            //make sure rotation exists
            ComponentQuery<IsBody, LocalToWorld> rotationMissingQuery = new(world);
            rotationMissingQuery.ExcludeComponent<Rotation>();
            foreach (var r in rotationMissingQuery)
            {
                ref IsBody body = ref r.component1;
                if (body.type != BodyType.Static)
                {
                    operation.SelectEntity(r.entity);
                    changed = true;
                }
            }

            if (changed)
            {
                operation.AddComponent<Rotation>(schema);
                operation.ClearSelection();
            }

            if (operation.Count > 0)
            {
                world.Perform(operation);
                operation.Clear();
            }
        }

        public readonly void PerformRaycastRequest(RaycastRequest raycast)
        {
            BepuPhysics.Simulation simulation = bepuSimulation;
            RaycastHandler handler = new(hits, this);
            simulation.RayCast(raycast.origin, raycast.direction, raycast.distance, ref handler);
            if (raycast.callback != default)
            {
                raycast.callback.Invoke(raycast.world, raycast, hits.AsSpan());
            }

            hits.Clear();
        }

        public readonly uint GetPhysicsEntity(int handle, bool isStatic)
        {
            return handleToBody[(handle, isStatic)];
        }

        private readonly void CreateAndDestroyPhysicsObjects()
        {
            //remove shapes and bodies of entities that dont exist anymore
            toRemove.Clear();
            BepuPhysics.Simulation simulation = bepuSimulation;
            foreach (uint bodyEntity in bodies.Keys)
            {
                if (!world.ContainsEntity(bodyEntity))
                {
                    CompiledBody body = bodies[bodyEntity];
                    bool isStatic = body.type == BodyType.Static;
                    handleToBody.Remove((body.handle, isStatic));
                    if (isStatic)
                    {
                        simulation.Statics.Remove(body.StaticBody);
                    }
                    else
                    {
                        simulation.Bodies.Remove(body.DynamicBody);
                    }

                    physicsObjectState[bodyEntity] = default;
                    body.Dispose();
                    toRemove.Add(bodyEntity);
                }
            }

            foreach (uint bodyEntity in toRemove)
            {
                bodies.Remove(bodyEntity);
            }

            //create shapes and bodies for entities that dont have them yet
            uint capacity = Allocations.GetNextPowerOf2(world.MaxEntityValue + 1);
            if (physicsObjectState.Length < capacity)
            {
                physicsObjectState.Length = capacity;
            }

            usedShapes.Clear();

            ComponentQuery<IsBody, LocalToWorld, WorldRotation> bodyQuery = new(world);
            foreach (var r in bodyQuery)
            {
                ref IsBody body = ref r.component1;
                ref LocalToWorld ltw = ref r.component2;
                ref WorldRotation worldRotation = ref r.component3;
                uint entity = r.entity;
                Shape shape = body.shape;
                if (shape.type == default)
                {
                    throw new Exception($"Physics body `{entity}` references invalid shape");
                }

                //make sure a reusable shape exists for this combination of (type, offset, mass, scale)
                BodyType type = body.type;
                bool newShape = false;
                float mass;
                if (type == BodyType.Static)
                {
                    mass = float.MaxValue;
                }
                else if (type == BodyType.Dynamic)
                {
                    mass = world.GetComponent<Mass>(entity).value;
                }
                else if (type == BodyType.Kinematic)
                {
                    mass = float.MaxValue;
                }
                else
                {
                    throw new Exception($"Physics body `{entity}` has an unrecognized body type `{type}`");
                }

                (Vector3 worldPosition, Quaternion ltwRotation, Vector3 scale) = ltw.Decomposed;
                Vector3 roundedScale = new(MathF.Round(scale.X, 3), MathF.Round(scale.Y, 3), MathF.Round(scale.Z, 3));
                Vector3 localOffset = shape.offset;
                uint shapeHash = GetHash(shape, type, roundedScale, mass);
                if (!shapes.TryGetValue(shapeHash, out CompiledShape compiledShape))
                {
                    compiledShape = CreateShape(shape, scale, mass);
                    shapes.Add(shapeHash, compiledShape);
                    newShape = true;
                }

                usedShapes.Add(shapeHash);

                //make sure a physics body exists for this combination of (shape, type)
                bool isStatic = type == BodyType.Static;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltwRotation);
                Vector3 desiredWorldPosition = worldPosition + worldOffset;
                Quaternion desiredWorldRotation = worldRotation.value;
                if (!bodies.TryGetValue(entity, out CompiledBody compiledBody))
                {
                    compiledBody = CreateBody(body, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    bodies.Add(entity, compiledBody);
                    handleToBody.Add((compiledBody.handle, isStatic), entity);
                }
                else if (compiledBody.version != body.version || compiledBody.type != type || newShape)
                {
                    handleToBody.Remove((compiledBody.handle, isStatic));
                    if (isStatic)
                    {
                        bepuSimulation.RemoveStaticBody(compiledBody.StaticBody);
                    }
                    else
                    {
                        bepuSimulation.RemoveDynamicBody(compiledBody.DynamicBody);
                    }

                    compiledBody.Dispose();
                    compiledBody = CreateBody(body, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    isStatic = compiledBody.type == BodyType.Static;
                    bodies[entity] = compiledBody;
                    handleToBody.Add((compiledBody.handle, isStatic), entity);
                }

                //copy values from entity onto physics object (if different from last known state)
                ref BodyState state = ref physicsObjectState[entity];
                if (isStatic)
                {
                    StaticReference staticReference = bepuSimulation.GetStaticBody(compiledBody.StaticBody);
                    if (state.position != desiredWorldPosition || state.rotation != desiredWorldRotation)
                    {
                        state.position = desiredWorldPosition;
                        state.rotation = desiredWorldRotation;
                        StaticDescription newDescription = staticReference.GetDescription();
                        newDescription.Pose.Position = desiredWorldPosition;
                        newDescription.Pose.Orientation = desiredWorldRotation;
                        staticReference.ApplyDescription(newDescription);
                    }
                }
                else
                {
                    BodyReference bodyReference = bepuSimulation.GetDynamicBody(compiledBody.DynamicBody);
                    if (state.position != desiredWorldPosition || state.rotation != desiredWorldRotation)
                    {
                        ref RigidPose pose = ref bodyReference.Pose;
                        pose.Position = desiredWorldPosition;
                        pose.Orientation = desiredWorldRotation;
                        bodyReference.Awake = true;
                    }

                    Vector3 linearVelocity = world.GetComponent(entity, new LinearVelocity()).value; //optional
                    Vector3 angularVelocity = world.GetComponent(entity, new AngularVelocity()).value; //optional
                    if (state.linearVelocity != linearVelocity || state.angularVelocity != angularVelocity)
                    {
                        ref BodyVelocity velocity = ref bodyReference.Velocity;
                        velocity.Linear = linearVelocity;
                        velocity.Angular = angularVelocity;
                        bodyReference.Awake = true;
                    }
                }
            }

            //remove shapes that are no longer used
            toRemove.Clear();
            foreach (uint shapeHash in shapes.Keys)
            {
                if (!usedShapes.Contains(shapeHash))
                {
                    toRemove.Add(shapeHash);
                }
            }

            foreach (uint shapeHash in toRemove)
            {
                CompiledShape shape = shapes.Remove(shapeHash);
                simulation.Shapes.Remove(shape.shapeIndex);
                shape.Dispose();
            }
        }

        private readonly void CopyPhysicsObjectStateToEntities()
        {
            BepuPhysics.Simulation simulation = bepuSimulation;
            foreach (uint bodyEntity in bodies.Keys)
            {
                CompiledBody body = bodies[bodyEntity];
                Shape shape = world.GetComponent<IsBody>(bodyEntity).shape;
                bool isStatic = body.type == BodyType.Static;
                LocalToWorld ltw = world.GetComponent<LocalToWorld>(bodyEntity);
                Vector3 localOffset = shape.offset;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltw.Rotation);
                if (isStatic)
                {
                    StaticReference staticReference = simulation.Statics[body.StaticBody];
                    StaticDescription description = staticReference.GetDescription();
                    Vector3 finalWorldPosition = description.Pose.Position - worldOffset;
                    Quaternion finalWorldRotation = description.Pose.Orientation;
                    physicsObjectState[bodyEntity] = new(finalWorldPosition, finalWorldRotation, Vector3.Zero, Vector3.Zero);

                    //copy bounds
                    if (!world.ContainsComponent<WorldBounds>(bodyEntity))
                    {
                        world.AddComponent(bodyEntity, new WorldBounds(staticReference.BoundingBox.Min, staticReference.BoundingBox.Max));
                    }
                    else
                    {
                        ref WorldBounds bounds = ref world.GetComponent<WorldBounds>(bodyEntity);
                        bounds.min = staticReference.BoundingBox.Min;
                        bounds.max = staticReference.BoundingBox.Max;
                    }
                }
                else
                {
                    uint bodyParent = world.GetParent(bodyEntity);
                    Matrix4x4 wtl = Matrix4x4.Identity;
                    if (bodyParent != default)
                    {
                        Matrix4x4.Invert(world.GetComponent(bodyParent, LocalToWorld.Default).value, out wtl);
                    }

                    //copy into individual local components
                    BodyReference bodyReference = simulation.Bodies[body.DynamicBody];
                    ref RigidPose pose = ref bodyReference.Pose;
                    ref Position localPosition = ref world.GetComponent<Position>(bodyEntity);
                    Vector3 finalWorldPosition = pose.Position - worldOffset;
                    localPosition.value = Vector3.Transform(finalWorldPosition, wtl);

                    ref Rotation localRotation = ref world.GetComponent<Rotation>(bodyEntity);
                    Quaternion finalWorldRotation = pose.Orientation;
                    if (bodyParent != default)
                    {
                        localRotation.value = Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(wtl) * finalWorldRotation);
                    }
                    else
                    {
                        localRotation.value = finalWorldRotation;
                    }

                    ref LinearVelocity linearVelocity = ref world.GetComponent<LinearVelocity>(bodyEntity);
                    BodyVelocity velocity = bodyReference.Velocity;
                    linearVelocity.value = velocity.Linear;

                    ref AngularVelocity angularVelocity = ref world.GetComponent<AngularVelocity>(bodyEntity);
                    angularVelocity.value = velocity.Angular;
                    physicsObjectState[bodyEntity] = new(finalWorldPosition, finalWorldRotation, velocity.Linear, velocity.Angular);

                    //copy bounds
                    if (!world.ContainsComponent<WorldBounds>(bodyEntity))
                    {
                        world.AddComponent(bodyEntity, new WorldBounds(bodyReference.BoundingBox.Min, bodyReference.BoundingBox.Max));
                    }
                    else
                    {
                        ref WorldBounds bounds = ref world.GetComponent<WorldBounds>(bodyEntity);
                        bounds.min = bodyReference.BoundingBox.Min;
                        bounds.max = bodyReference.BoundingBox.Max;
                    }
                }
            }
        }

        private readonly CompiledShape CreateShape(Shape shape, Vector3 scale, float mass)
        {
            BepuPhysics.Simulation simulation = bepuSimulation;
            Vector3 offset = shape.offset;
            if (shape.Is(out CubeShape cube))
            {
                Vector3 extents = cube.extents * scale;
                Box box = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                TypedIndex shapeIndex = simulation.Shapes.Add(box);
                BodyInertia bodyInertia = box.ComputeInertia(mass);
                return new CompiledShape(offset, scale, shapeIndex, bodyInertia);
            }
            else if (shape.Is(out SphereShape circle))
            {
                float radius = circle.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                Sphere sphere = new(radius);
                TypedIndex shapeIndex = simulation.Shapes.Add(sphere);
                BodyInertia bodyInertia = sphere.ComputeInertia(mass);
                return new CompiledShape(offset, scale, shapeIndex, bodyInertia);
            }
            else
            {
                throw new Exception($"Shape `{shape}` is not a known one");
            }
        }

        private readonly CompiledBody CreateBody(IsBody bodyComponent, CompiledShape shape, Vector3 worldPosition, Quaternion worldRotation)
        {
            BepuPhysics.Simulation simulation = bepuSimulation;
            BodyType type = bodyComponent.type;
            RigidPose pose = new(worldPosition, worldRotation);
            ContinuousDetection continuity = ContinuousDetection.Discrete;
            BodyActivityDescription activity = new(0.01f);
            uint version = bodyComponent.version;
            int handle;
            if (type == BodyType.Dynamic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateDynamic(pose, default, shape.bodyInertia, collidable, activity);
                BodyHandle bodyHandle = simulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == BodyType.Kinematic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateKinematic(pose, default, collidable, activity);
                BodyHandle bodyHandle = simulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == BodyType.Static)
            {
                StaticDescription description = new(pose, shape.shapeIndex, continuity);
                StaticHandle staticHandle = simulation.Statics.Add(description);
                handle = staticHandle.Value;
            }
            else
            {
                throw new Exception($"Physics body `{bodyComponent}` has an unrecognized body type `{type}`");
            }

            return new(version, handle, type);
        }

        private readonly Vector3 GetGlobalGravity()
        {
            Vector3 totalGravity = default;
            ComponentQuery<IsGravitySource, LocalToWorld> directionalGravityQuery = new(world);
            directionalGravityQuery.RequireTag<IsDirectionalGravity>();
            foreach (var r in directionalGravityQuery)
            {
                ref IsGravitySource gravitySource = ref r.component1;
                ref LocalToWorld ltw = ref r.component2;
                totalGravity += ltw.Forward * gravitySource.force;
            }

            return totalGravity;
        }

        private static uint GetHash(Shape shape, BodyType type, Vector3 scale, float mass)
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + shape.GetHashCode();
                hash = hash * 23 + type.GetHashCode();
                hash = hash * 23 + scale.GetHashCode();
                hash = hash * 23 + mass.GetHashCode();
                return (uint)hash;
            }
        }
    }
}