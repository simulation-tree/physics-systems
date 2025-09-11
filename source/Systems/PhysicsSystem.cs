using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using Collections.Generic;
using Physics.Components;
using Physics.Messages;
using Shapes;
using Shapes.Types;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Worlds;

namespace Physics.Systems
{
    public partial class PhysicsSystem : SystemBase, IListener<RaycastRequest>, IListener<PhysicsUpdate>
    {
        private readonly World world;
        private readonly List<RaycastHit> hits;
        private readonly List<uint> toRemove;
        private readonly List<uint> usedShapes;
        private readonly Array<PhysicsObjectState> physicsObjectStates;
        private readonly Dictionary<uint, CompiledBody> bodies;
        private readonly Dictionary<uint, CompiledShape> shapes;
        private readonly Dictionary<(int, bool), uint> handleToBody;
        private readonly List<PointGravitySource> pointGravitySources;
        private readonly BepuPhysics.Simulation bepuSimulation;
        private readonly BepuBufferPool bufferPool;
        private readonly MemoryAddress gravity;
        private readonly Operation operation;
        private readonly int bodyType;
        private readonly int ltwType;
        private readonly int gravitySourceType;
        private readonly int positionType;
        private readonly int rotationType;
        private readonly int linearVelocityType;
        private readonly int angularVelocityType;
        private readonly int directionalGravityTag;
        private readonly int pointGravityType;
        private RaycastHandler handler;

        public PhysicsSystem(Simulator simulator, World world) : base(simulator)
        {
            this.world = world;
            bufferPool = new();
            gravity = MemoryAddress.AllocateValue(new Vector3(0f));
            NarrowPhaseCallbacks narrowPhaseCallbacks = new(new SpringSettings(30, 1));
            PoseIntegratorCallbacks poseIntegratorCallbacks = new(gravity, 0f, 0f);
            SolveDescription solveDescription = new(8, 1);
            bepuSimulation = BepuPhysics.Simulation.Create(bufferPool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription);

            handler = new(hits, this);
            hits = new();
            toRemove = new();
            usedShapes = new();
            physicsObjectStates = new();
            bodies = new();
            shapes = new();
            handleToBody = new();
            pointGravitySources = new();
            operation = new(world);

            Schema schema = world.Schema;
            bodyType = schema.GetComponentType<IsBody>();
            ltwType = schema.GetComponentType<LocalToWorld>();
            gravitySourceType = schema.GetComponentType<IsGravitySource>();
            positionType = schema.GetComponentType<Position>();
            rotationType = schema.GetComponentType<Rotation>();
            linearVelocityType = schema.GetComponentType<LinearVelocity>();
            angularVelocityType = schema.GetComponentType<AngularVelocity>();
            directionalGravityTag = schema.GetTagType<IsDirectionalGravity>();
            pointGravityType = schema.GetComponentType<IsPointGravity>();
        }

        public override void Dispose()
        {
            operation.Dispose();

            foreach (CompiledBody body in bodies.Values)
            {
                body.Dispose();
            }

            foreach (CompiledShape shape in shapes.Values)
            {
                shape.Dispose();
            }

            pointGravitySources.Dispose();
            handleToBody.Dispose();
            shapes.Dispose();
            bodies.Dispose();
            physicsObjectStates.Dispose();
            bepuSimulation.Dispose();
            bufferPool.Dispose();
            gravity.Dispose();
            usedShapes.Dispose();
            toRemove.Dispose();
            hits.Dispose();
        }

        void IListener<RaycastRequest>.Receive(ref RaycastRequest raycast)
        {
            bepuSimulation.RayCast(raycast.origin, raycast.direction, raycast.distance, bufferPool, ref handler);
            if (raycast.callback != default)
            {
                //todo: whats the point of raycasting if theres no callback given?
                raycast.callback.Invoke(world, raycast, hits.AsSpan());
            }

            hits.Clear();
        }

        void IListener<PhysicsUpdate>.Receive(ref PhysicsUpdate message)
        {
            gravity.Write(GetGlobalGravity());
            AddMissingComponents();
            ApplyPointGravity(message.deltaTime);

            // create shapes and bodies for entities that dont have them yet
            int capacity = (world.MaxEntityValue + 1).GetNextPowerOf2();
            int currentLength = this.physicsObjectStates.Length;
            if (currentLength < capacity)
            {
                this.physicsObjectStates.Length = capacity;

                // reset the new entires to default
                for (int i = currentLength; i < capacity; i++)
                {
                    this.physicsObjectStates[i] = default;
                }
            }

            Span<PhysicsObjectState> physicsObjectStates = this.physicsObjectStates.AsSpan();
            CreateAndDestroyPhysicsObjects(physicsObjectStates);

            if (message.deltaTime > 0)
            {
                bepuSimulation.Timestep((float)message.deltaTime);
            }

            CopyPhysicsObjectStateToEntities(physicsObjectStates);
        }

        private void ApplyPointGravity(double deltaTime)
        {
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            BitMask bodyComponents = new(bodyType, ltwType, linearVelocityType);
            BitMask pointGravityComponents = new(gravitySourceType, ltwType, pointGravityType);

            //find point gravity sources first
            pointGravitySources.Clear();
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                Definition definition = chunk.Definition;
                if (definition.IsEnabled && definition.componentTypes.ContainsAll(pointGravityComponents))
                {
                    ComponentEnumerator<IsGravitySource> gravitySources = chunk.GetComponents<IsGravitySource>(gravitySourceType);
                    ComponentEnumerator<LocalToWorld> ltws = chunk.GetComponents<LocalToWorld>(ltwType);
                    ComponentEnumerator<IsPointGravity> pointGravities = chunk.GetComponents<IsPointGravity>(pointGravityType);
                    for (int i = 0; i < chunk.Count; i++)
                    {
                        ref IsGravitySource gravitySource = ref gravitySources[i];
                        ref LocalToWorld ltw = ref ltws[i];
                        ref IsPointGravity pointGravity = ref pointGravities[i];
                        pointGravitySources.Add(new(ltw.Position, gravitySource.force, pointGravity.radius));
                    }
                }
            }

            //apply gravity
            Span<PointGravitySource> pointGravitySourcesSpan = pointGravitySources.AsSpan();
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                Definition definition = chunk.Definition;
                if (definition.IsEnabled && definition.componentTypes.ContainsAll(bodyComponents))
                {
                    int entityCount = chunk.Count;
                    ComponentEnumerator<IsBody> bodies = chunk.GetComponents<IsBody>(bodyType);
                    ComponentEnumerator<LocalToWorld> ltws = chunk.GetComponents<LocalToWorld>(ltwType);
                    ComponentEnumerator<LinearVelocity> linearVelocities = chunk.GetComponents<LinearVelocity>(linearVelocityType);
                    for (int i = 0; i < entityCount; i++)
                    {
                        ref IsBody body = ref bodies[i];
                        ref LocalToWorld ltw = ref ltws[i];
                        ref LinearVelocity linearVelocity = ref linearVelocities[i];
                        if (body.type == BodyType.Dynamic)
                        {
                            Vector3 accumulatedGravity = default;
                            Vector3 worldPosition = ltw.Position;
                            for (int s = 0; s < pointGravitySourcesSpan.Length; s++)
                            {
                                PointGravitySource pointGravitySource = pointGravitySourcesSpan[s];
                                Vector3 offset = worldPosition - pointGravitySource.position;
                                float distanceSquared = offset.LengthSquared();
                                if (distanceSquared < pointGravitySource.radiusSquared)
                                {
                                    float distance = MathF.Sqrt(distanceSquared);
                                    float attenuation = 1f - MathF.Min(distance / pointGravitySource.radius, 1f);
                                    Vector3 direction = Vector3.Normalize(offset);
                                    accumulatedGravity += direction * pointGravitySource.force * attenuation;
                                }
                            }

                            if (accumulatedGravity != default)
                            {
                                accumulatedGravity *= -1;
                                linearVelocity.value += accumulatedGravity * (float)deltaTime;
                            }
                        }
                    }
                }
            }
        }

        private void AddMissingComponents()
        {
            //make sure linear velocity exists
            //todo: decide if detecting dynamic bodies should be possible with a tag, because thatd make this cheaper
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            BitMask dynamicComponentTypes = new(linearVelocityType, angularVelocityType, positionType, rotationType);
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                BitMask componentTypes = chunk.Definition.componentTypes;
                if (componentTypes.Contains(bodyType) && !componentTypes.ContainsAny(dynamicComponentTypes))
                {
                    ComponentEnumerator<IsBody> bodies = chunk.GetComponents<IsBody>(bodyType);
                    int entityCount = chunk.Count;
                    for (int i = 0; i < entityCount; i++)
                    {
                        ref IsBody body = ref bodies[i];
                        if (body.type == BodyType.Dynamic)
                        {
                            operation.AppendEntityToSelection(chunk.Entities[i]);
                        }
                    }
                }
            }

            if (operation.Count > 0)
            {
                operation.TryAddComponentType(linearVelocityType);
                operation.TryAddComponentType(angularVelocityType);
                operation.TryAddComponentType(positionType);
                operation.TryAddComponentType(rotationType);
                operation.Perform();
                operation.Reset();
            }
        }

        public uint GetPhysicsEntity(int handle, bool isStatic)
        {
            return handleToBody[(handle, isStatic)];
        }

        private void CreateAndDestroyPhysicsObjects(Span<PhysicsObjectState> physicsObjectStates)
        {
            //remove shapes and bodies of entities that dont exist anymore
            toRemove.Clear();
            foreach (uint bodyEntity in bodies.Keys)
            {
                if (!world.ContainsEntity(bodyEntity))
                {
                    CompiledBody body = bodies[bodyEntity];
                    bool isStatic = body.type == BodyType.Static;
                    handleToBody.Remove((body.handle, isStatic));
                    if (isStatic)
                    {
                        bepuSimulation.Statics.Remove(body.StaticBody);
                    }
                    else
                    {
                        bepuSimulation.Bodies.Remove(body.DynamicBody);
                    }

                    physicsObjectStates[(int)bodyEntity] = default;
                    body.Dispose();
                    toRemove.Add(bodyEntity);
                }
            }

            foreach (uint bodyEntity in toRemove)
            {
                bodies.Remove(bodyEntity);
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
                if (shape.TypeIndex == default)
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
                Vector3 localOffset = body.offset;
                uint shapeHash = GetHash(shape, localOffset, type, roundedScale, mass);
                if (!shapes.TryGetValue(shapeHash, out CompiledShape compiledShape))
                {
                    compiledShape = CreateShape(shape, localOffset, scale, mass);
                    shapes.Add(shapeHash, compiledShape);
                    newShape = true;
                }

                usedShapes.Add(shapeHash);

                //make sure a physics body exists for this combination of (shape, type)
                ref PhysicsObjectState state = ref physicsObjectStates[(int)entity];
                bool isStatic = type == BodyType.Static;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltwRotation);
                Vector3 desiredWorldPosition = worldPosition + worldOffset;
                Quaternion desiredWorldRotation = worldRotation.value;
                if (!bodies.TryGetValue(entity, out CompiledBody compiledBody))
                {
                    compiledBody = CreateBody(body, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    bodies.Add(entity, compiledBody);
                    handleToBody.Add((compiledBody.handle, isStatic), entity);
                    state.position = desiredWorldPosition;
                    state.rotation = desiredWorldRotation;
                    state.linearVelocity = Vector3.Zero;
                    state.angularVelocity = Vector3.Zero;
                }
                else if (compiledBody.version != body.version || compiledBody.type != type || newShape)
                {
                    handleToBody.Remove((compiledBody.handle, isStatic));
                    if (isStatic)
                    {
                        bepuSimulation.Statics.Remove(compiledBody.StaticBody);
                    }
                    else
                    {
                        bepuSimulation.Bodies.Remove(compiledBody.DynamicBody);
                    }

                    compiledBody.Dispose();
                    compiledBody = CreateBody(body, compiledShape, desiredWorldPosition, desiredWorldRotation);
                    isStatic = compiledBody.type == BodyType.Static;
                    bodies[entity] = compiledBody;
                    handleToBody.Add((compiledBody.handle, isStatic), entity);
                    state.position = desiredWorldPosition;
                    state.rotation = desiredWorldRotation;
                    state.linearVelocity = Vector3.Zero;
                    state.angularVelocity = Vector3.Zero;
                }

                //copy values from entity onto physics object (if different from last known state)
                if (isStatic)
                {
                    StaticReference staticReference = bepuSimulation.Statics.GetStaticReference(compiledBody.StaticBody);
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
                    BodyReference bodyReference = bepuSimulation.Bodies.GetBodyReference(compiledBody.DynamicBody);
                    if (state.position != desiredWorldPosition || state.rotation != desiredWorldRotation)
                    {
                        ref RigidPose pose = ref bodyReference.Pose;
                        pose.Position = desiredWorldPosition;
                        pose.Orientation = desiredWorldRotation;
                        bodyReference.Awake = true;
                    }

                    Vector3 linearVelocity = world.GetComponentOrDefault(entity, new LinearVelocity()).value; //optional
                    Vector3 angularVelocity = world.GetComponentOrDefault(entity, new AngularVelocity()).value; //optional
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
                shapes.Remove(shapeHash, out CompiledShape shape);
                bepuSimulation.Shapes.Remove(shape.shapeIndex);
                shape.Dispose();
            }
        }

        private void CopyPhysicsObjectStateToEntities(Span<PhysicsObjectState> physicsObjectStates)
        {
            foreach (uint bodyEntity in bodies.Keys)
            {
                CompiledBody body = bodies[bodyEntity];
                IsBody component = world.GetComponent<IsBody>(bodyEntity, bodyType);
                bool isStatic = body.type == BodyType.Static;
                LocalToWorld ltw = world.GetComponent<LocalToWorld>(bodyEntity, ltwType);
                Vector3 localOffset = component.offset;
                Vector3 worldOffset = Vector3.Transform(localOffset, ltw.Rotation);
                if (isStatic)
                {
                    StaticReference staticReference = bepuSimulation.Statics[body.StaticBody];
                    StaticDescription description = staticReference.GetDescription();
                    Vector3 finalWorldPosition = description.Pose.Position - worldOffset;
                    Quaternion finalWorldRotation = description.Pose.Orientation;
                    physicsObjectStates[(int)bodyEntity] = new(finalWorldPosition, finalWorldRotation, Vector3.Zero, Vector3.Zero);

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
                        Matrix4x4.Invert(world.GetComponentOrDefault(bodyParent, LocalToWorld.Default).value, out wtl);
                    }

                    //copy into individual local components
                    BodyReference bodyReference = bepuSimulation.Bodies[body.DynamicBody];
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
                    physicsObjectStates[(int)bodyEntity] = new(finalWorldPosition, finalWorldRotation, velocity.Linear, velocity.Angular);

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

        private CompiledShape CreateShape(Shape shape, Vector3 offset, Vector3 scale, float mass)
        {
            if (shape.Is(out CubeShape cube))
            {
                Vector3 extents = cube.extents * scale;
                Box box = new(extents.X * 2, extents.Y * 2, extents.Z * 2);
                TypedIndex shapeIndex = bepuSimulation.Shapes.Add(box);
                BodyInertia bodyInertia = box.ComputeInertia(mass);
                return new CompiledShape(offset, scale, shapeIndex, bodyInertia);
            }
            else if (shape.Is(out SphereShape circle))
            {
                float radius = circle.radius * MathF.Max(scale.X, MathF.Max(scale.Y, scale.Z));
                Sphere sphere = new(radius);
                TypedIndex shapeIndex = bepuSimulation.Shapes.Add(sphere);
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
                BodyHandle bodyHandle = bepuSimulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == BodyType.Kinematic)
            {
                CollidableDescription collidable = new(shape.shapeIndex, 10f, continuity);
                BodyDescription description = BodyDescription.CreateKinematic(pose, default, collidable, activity);
                BodyHandle bodyHandle = bepuSimulation.Bodies.Add(description);
                handle = bodyHandle.Value;
            }
            else if (type == BodyType.Static)
            {
                StaticDescription description = new(pose, shape.shapeIndex, continuity);
                StaticHandle staticHandle = bepuSimulation.Statics.Add(description);
                handle = staticHandle.Value;
            }
            else
            {
                throw new Exception($"Physics body `{bodyComponent}` has an unrecognized body type `{type}`");
            }

            return new(version, handle, type);
        }

        private Vector3 GetGlobalGravity()
        {
            Vector3 totalGravity = default;
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            BitMask componentTypes = new(gravitySourceType, ltwType);
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                Definition definition = chunk.Definition;
                if (!definition.IsDisabled && definition.tagTypes.Contains(directionalGravityTag) && definition.componentTypes.ContainsAll(componentTypes))
                {
                    ComponentEnumerator<IsGravitySource> gravitySources = chunk.GetComponents<IsGravitySource>(gravitySourceType);
                    ComponentEnumerator<LocalToWorld> ltws = chunk.GetComponents<LocalToWorld>(ltwType);
                    for (int i = 0; i < chunk.Count; i++)
                    {
                        ref IsGravitySource gravitySource = ref gravitySources[i];
                        ref LocalToWorld ltw = ref ltws[i];
                        totalGravity += ltw.Forward * gravitySource.force;
                    }
                }
            }

            return totalGravity;
        }

        private static uint GetHash(Shape shape, Vector3 offset, BodyType type, Vector3 scale, float mass)
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + shape.GetHashCode();
                hash = hash * 23 + offset.GetHashCode();
                hash = hash * 23 + type.GetHashCode();
                hash = hash * 23 + scale.GetHashCode();
                hash = hash * 23 + mass.GetHashCode();
                return (uint)hash;
            }
        }
    }
}