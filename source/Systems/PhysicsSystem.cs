using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using Collections;
using Physics.Components;
using Physics.Events;
using Simulation;
using Simulation.Functions;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Transforms.Components;
using Unmanaged;

namespace Physics.Systems
{
    public readonly struct PhysicsSystem : ISystem
    {
        private readonly List<RaycastHit> hits;
        private readonly List<Raycast> raycasts;
        private readonly ComponentQuery<IsBody, LocalToWorld, WorldRotation> bodyQuery;
        private readonly ComponentQuery<IsBody, LinearVelocity> bodyLinearVelocityQuery;
        private readonly ComponentQuery<IsGravitySource, IsDirectionalGravity> directionalGravityQuery;
        private readonly ComponentQuery<IsGravitySource, IsPointGravity, LocalToWorld> pointGravityQuery;
        private readonly Array<BodyState> physicsObjectState;
        private readonly Dictionary<uint, CompiledBody> bodies;
        private readonly Dictionary<int, CompiledShape> shapes;
        private readonly Dictionary<(int, bool), uint> handleToBody;
        private readonly BepuSimulation physicsSimulation;
        private readonly BepuBufferPool bufferPool;
        private readonly Allocation gravity;

        readonly unsafe InitializeFunction ISystem.Initialize => new(&Initialize);
        readonly unsafe IterateFunction ISystem.Iterate => new(&Update);
        readonly unsafe FinalizeFunction ISystem.Finalize => new(&Finalize);

        readonly unsafe uint ISystem.GetMessageHandlers(USpan<MessageHandler> buffer)
        {
            buffer[0] = MessageHandler.Create<Raycast>(new(&HandleRaycast));
            return 1;
        }

        [UnmanagedCallersOnly]
        private static void Initialize(SystemContainer container, World world)
        {
        }

        [UnmanagedCallersOnly]
        private static void Update(SystemContainer container, World world, TimeSpan delta)
        {
            ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
            system.Update(world, delta);
        }

        [UnmanagedCallersOnly]
        private static void Finalize(SystemContainer container, World world)
        {
            if (container.World == world)
            {
                ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
                system.Dispose();
            }
        }

        [UnmanagedCallersOnly]
        private static void HandleRaycast(SystemContainer container, World world, Allocation message)
        {
            ref PhysicsSystem system = ref container.Read<PhysicsSystem>();
            Raycast raycast = message.Read<Raycast>();
            system.PerformRaycastRequest(world, raycast);
        }

        public PhysicsSystem()
        {
            bufferPool = new();
            gravity = Allocation.Create(new Vector3(0f));
            NarrowPhaseCallbacks narrowPhaseCallbacks = new(new SpringSettings(30, 1));
            PoseIntegratorCallbacks poseIntegratorCallbacks = new(gravity, 0f, 0f);
            SolveDescription solveDescription = new(8, 1);
            physicsSimulation = new(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);

            hits = new();
            raycasts = new();
            bodyQuery = new();
            bodyLinearVelocityQuery = new();
            directionalGravityQuery = new();
            pointGravityQuery = new();
            physicsObjectState = new();
            bodies = new();
            shapes = new();
            handleToBody = new();
        }

        private void Dispose()
        {
            foreach (uint bodyEntity in bodies.Keys)
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
            pointGravityQuery.Dispose();
            directionalGravityQuery.Dispose();
            bodyLinearVelocityQuery.Dispose();
            physicsObjectState.Dispose();
            bodyQuery.Dispose();
            physicsSimulation.Dispose();
            bufferPool.Dispose();
            gravity.Dispose();
            raycasts.Dispose();
            hits.Dispose();
        }

        private void Update(World world, TimeSpan delta)
        {
            Vector3 globalGravity = GetGlobalGravity(world);
            gravity.Write(0, globalGravity);
            ApplyPointGravity(world, delta);
            CreateAndDestroyPhysicsObjects(world);

            if (delta.Ticks > 0)
            {
                physicsSimulation.Update(delta);
            }

            CopyPhysicsObjectStateToEntities(world);
            PerformRaycastRequests(world);
        }

        private void ApplyPointGravity(World world, TimeSpan delta)
        {
            //todo: fault: this needs a test to verify, but in theory it should work
            pointGravityQuery.Update(world);
            using Array<(Vector3, float, float)> pointGravitySources = new(pointGravityQuery.Count);
            uint index = 0;
            foreach (var x in pointGravityQuery)
            {
                float force = x.Component1.force;
                float radius = x.Component2.radius;
                Vector3 worldPosition = x.Component3.Position;
                pointGravitySources[index] = (worldPosition, force, radius);
                index++;
            }

            bodyQuery.Update(world);
            foreach (var x in bodyQuery)
            {
                uint bodyEntity = x.entity;
                IsBody bodyComponent = x.Component1;
                if (bodyComponent.type == IsBody.Type.Dynamic)
                {
                    Vector3 accumulatedGravity = default;
                    Vector3 worldPosition = x.Component2.Position;
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
                        if (!world.ContainsComponent<LinearVelocity>(bodyEntity))
                        {
                            world.AddComponent(bodyEntity, new LinearVelocity(accumulatedGravity * (float)delta.TotalSeconds));
                        }
                        else
                        {
                            ref LinearVelocity linearVelocity = ref world.GetComponentRef<LinearVelocity>(bodyEntity);
                            linearVelocity.value += accumulatedGravity * (float)delta.TotalSeconds;
                        }
                    }
                }
            }
        }

        private void AddRaycastRequest(Raycast raycast)
        {
            raycasts.Add(raycast);
        }

        private void PerformRaycastRequests(World world)
        {
            foreach (Raycast raycast in raycasts)
            {
                PerformRaycastRequest(world, raycast);
            }

            raycasts.Clear();
        }

        private void PerformRaycastRequest(World world, Raycast raycast)
        {
            BepuPhysics.Simulation simulation = physicsSimulation;
            RaycastHandler handler = new(hits, this);
            simulation.RayCast(raycast.origin, raycast.direction, raycast.distance, ref handler);
            if (raycast.callback != default)
            {
                raycast.callback.Invoke(world, raycast, hits.AsSpan());
            }

            hits.Clear();
        }

        public uint GetPhysicsEntity(int handle, bool isStatic)
        {
            return handleToBody[(handle, isStatic)];
        }

        private void CreateAndDestroyPhysicsObjects(World world)
        {
            //remove shapes and bodies of entities that dont exist anymore
            using List<uint> bodiesRemoved = new();
            BepuPhysics.Simulation simulation = physicsSimulation;
            foreach (uint bodyEntity in bodies.Keys)
            {
                if (!world.ContainsEntity(bodyEntity))
                {
                    CompiledBody body = bodies[bodyEntity];
                    int handle = body.handle;
                    bool isStatic = body.type == IsBody.Type.Static;
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
                    bodiesRemoved.Add(bodyEntity);
                }
            }

            foreach (uint bodyEntity in bodiesRemoved)
            {
                bodies.Remove(bodyEntity);
            }

            //create shapes and bodies for entities that dont have them yet
            physicsObjectState.Length = world.MaxEntityValue + 1;
            bodyQuery.Update(world);
            using List<int> usedShapes = new();
            foreach (var r in bodyQuery)
            {
                uint bodyEntity = r.entity;
                IsBody bodyComponent = r.Component1;
                LocalToWorld ltw = r.Component2;
                Shape shape = bodyComponent.shape;
                if (shape.type == default)
                {
                    throw new Exception($"Physics body `{bodyEntity}` references invalid shape");
                }

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

                (Vector3 worldPosition, Quaternion ltwRotation, Vector3 scale) = ltw.Decomposed;
                Vector3 roundedScale = new(MathF.Round(scale.X, 3), MathF.Round(scale.Y, 3), MathF.Round(scale.Z, 3));
                Vector3 localOffset = shape.offset;
                int shapeHash = HashCode.Combine(shape, type, roundedScale, mass);
                if (!shapes.TryGetValue(shapeHash, out CompiledShape compiledShape))
                {
                    compiledShape = CreateShape(shape, scale, mass);
                    shapes.Add(shapeHash, compiledShape);
                    newShape = true;
                }

                usedShapes.Add(shapeHash);

                //make sure a physics body exists for this combination of (shape, type)
                bool isStatic = type == IsBody.Type.Static;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltwRotation);
                Vector3 desiredWorldPosition = worldPosition + worldOffset;
                Quaternion desiredWorldRotation = r.Component3.value;
                if (!bodies.TryGetValue(bodyEntity, out CompiledBody compiledBody))
                {
                    compiledBody = CreateBody(bodyComponent, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    bodies.Add(bodyEntity, compiledBody);
                    handleToBody.Add((compiledBody.handle, isStatic), bodyEntity);
                }
                else if (compiledBody.version != bodyComponent.version || compiledBody.type != type || newShape)
                {
                    handleToBody.Remove((compiledBody.handle, isStatic));
                    if (isStatic)
                    {
                        simulation.Statics.Remove(compiledBody.StaticBody);
                    }
                    else
                    {
                        simulation.Bodies.Remove(compiledBody.DynamicBody);
                    }

                    compiledBody.Dispose();
                    compiledBody = CreateBody(bodyComponent, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    isStatic = compiledBody.type == IsBody.Type.Static;
                    bodies[bodyEntity] = compiledBody;
                    handleToBody.Add((compiledBody.handle, isStatic), bodyEntity);
                }

                //copy values from entity onto physics object (if different from last known state)
                ref BodyState state = ref physicsObjectState[bodyEntity];
                if (isStatic)
                {
                    StaticReference staticReference = simulation.Statics[compiledBody.StaticBody];
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
                    BodyReference bodyReference = simulation.Bodies[compiledBody.DynamicBody];
                    if (state.position != desiredWorldPosition || state.rotation != desiredWorldRotation)
                    {
                        ref RigidPose pose = ref bodyReference.Pose;
                        pose.Position = desiredWorldPosition;
                        pose.Orientation = desiredWorldRotation;
                        bodyReference.Awake = true;
                    }

                    Vector3 linearVelocity = world.GetComponent(bodyEntity, new LinearVelocity()).value; //optional
                    Vector3 angularVelocity = world.GetComponent(bodyEntity, new AngularVelocity()).value; //optional
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
            using List<int> shapesToRemove = new();
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
                simulation.Shapes.Remove(shape.shapeIndex);
                shape.Dispose();
            }
        }

        private void CopyPhysicsObjectStateToEntities(World world)
        {
            BepuPhysics.Simulation simulation = physicsSimulation;
            foreach (uint bodyEntity in bodies.Keys)
            {
                CompiledBody body = bodies[bodyEntity];
                Shape shape = world.GetComponent<IsBody>(bodyEntity).shape;
                bool isStatic = body.type == IsBody.Type.Static;
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
                        ref WorldBounds bounds = ref world.GetComponentRef<WorldBounds>(bodyEntity);
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
                    physicsObjectState[bodyEntity] = new(finalWorldPosition, finalWorldRotation, velocity.Linear, velocity.Angular);

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

                    //calculate local to world
                }
            }
        }

        private CompiledShape CreateShape(Shape shape, Vector3 scale, float mass)
        {
            BepuPhysics.Simulation simulation = physicsSimulation;
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

        private CompiledBody CreateBody(IsBody bodyComponent, CompiledShape shape, Vector3 worldPosition, Quaternion worldRotation)
        {
            BepuPhysics.Simulation simulation = physicsSimulation;
            IsBody.Type type = bodyComponent.type;
            RigidPose pose = new(worldPosition, worldRotation);
            ContinuousDetection continuity = ContinuousDetection.Discrete;
            BodyActivityDescription activity = new(0.01f);
            uint version = bodyComponent.version;
            int handle;
            if (type == IsBody.Type.Dynamic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateDynamic(pose, default, shape.bodyInertia, collidable, activity);
                BodyHandle bodyHandle = simulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == IsBody.Type.Kinematic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateKinematic(pose, default, collidable, activity);
                BodyHandle bodyHandle = simulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == IsBody.Type.Static)
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

        private Vector3 GetGlobalGravity(World world)
        {
            Vector3 force = default;
            directionalGravityQuery.Update(world);
            foreach (var x in directionalGravityQuery)
            {
                LocalToWorld ltw = world.GetComponent<LocalToWorld>(x.entity);
                Vector3 forward = ltw.Forward;
                force += forward * x.Component1.force;
            }

            return force;
        }
    }
}