using Unity.Mathematics;
using UnityEngine;
using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

public partial class FluidSim {

    private float CellSize; //Size of each cell in the grid.
    private Particle[] ParticleCont; //Holds all the particles in the simulation.
    private Grid GridMap; //Holds all the information needed for the grid cell.

    private PreconditionedConjugateGradient PCG;
    private System.Diagnostics.Stopwatch SimulationTimer = new System.Diagnostics.Stopwatch();
    private bool GPUReadyFlag = false; //Flags when the simulation tells the GPU to update particle positions before rendering.

    private readonly bool TimeSimulation = false;
    private readonly bool TrackAngularMomentum = true;
    private readonly bool APICBoundarySafety = true;


    //Quickly converts 2D to 1D indices for accessing grid values.
    private int GridWidth, GridHeight, GridSize;
    float2 SimCentre;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Grid2DFlattenIndex(int x, int y) => y * GridWidth + x;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private (int, int) Grid1DUnfoldIndex(int n) => (n % GridWidth, n / GridWidth);




    private void RunSimlationStep() {
        deltaTime = Settings.TimeStep / 2; // For RK2.

        Parallel.For(0, ParticleCont.Length, i => {
            ParticleCont[i].CopyToBuffer();
        });

        // Checks if the user is clicking on the simulation. If so, apply a force to the particles, depending on the specific click.
        bool IsMouseClick = false;
        Vector2 mousePos = Vector2.zero;
        float MouseForceDirection = 0;
        if (Input.GetMouseButton(0) || Input.GetMouseButton(1)) {
            Settings.MouseGravity = Input.GetMouseButton(0) && Input.GetMouseButton(1);
            mousePos = Input.mousePosition;
            // Scales the mouse position to lie locally to the grid.
            mousePos.x = mousePos.x * CameraController.WorldWidth / (CellSize * Screen.width);
            mousePos.y = mousePos.y * CameraController.WorldHeight / (CellSize * Screen.height);
            if (mousePos.x < GridWidth && mousePos.y < GridHeight) {
                // Mouse is inside the bounds of the simulation.
                MouseForceDirection = Input.GetMouseButton(0) ? -1f : 1f;
                IsMouseClick = true;
            }
        } else {
            Settings.MouseGravity = false;
        }

        // Runs the simulation, first at position x, then at position x + v(x)*dt/2. This gives two sample points that will combine in RK2 discrete
        // integration to get the new particle position.
        float MaxVelocityThisStep = 0f;
        for (int k = 1; k <= 2; k++) {

            if (TimeSimulation) {
                Debug.Log("New Simulation Step:");
                SimulationTimer.Restart();
            }

            PCG.Clear(Settings.ProjectionStepSize);
            //Interpolates the grid velocities to the particles.
            InterpolateParticlesToGrid();


            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("Interpolation 1: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
                SimulationTimer.Restart();
            }


            //Updates particle velocities for external forces (gravity, collisions with objects, user controls).
            float gDirRad = Settings.gDirection / 180 * math.PI;
            float GravityStrength = Settings.gMagnitude * deltaTime;
            float2 SimCentre = float2.zero;
            if (AngularMomentumBoost) { SimCentre = new float2(GridWidth + 1f, GridHeight + 1f) / 2f; }

            Parallel.For(0, GridMap.Length(), i => {
                if (GridMap.Type[i] != 0) { //!Should this only be fluid cells???
                    if (Settings.MouseGravity) {
                        // Adds a tangential, rotational force to the fluid, to 'orbit' around the mouse, using the gravitational effect K/r^2.
                        AddForceToGrid((int)i, mousePos, GravityStrength, (d, t) => 1 / (d * d), UseTangentialVector: true);
                        // Pulls the fluid from all directions towards the fluid (constant, radial force).
                        AddForceToGrid((int)i, mousePos, GravityStrength, (d, t) => 1);
                    } else {
                        float2 GravityContribution = GravityStrength * new float2(math.cos(gDirRad), math.sin(gDirRad));
                        GridMap.Velocity_GP[i] += GravityContribution;
                        if (IsMouseClick) {
                            //Applies the mouse force to the grid.
                            Func<float, float, float> UserDistanceFunction = (d, t) => math.pow(1 - (d * d) / (t * t), 2); //represents the function (1-x^2)^2, where x = distance / maxdistance (threshold).
                            AddForceToGrid((int)i, mousePos, Settings.MouseStrength * MouseForceDirection, UserDistanceFunction, Settings.MouseRadius);
                        }
                    }

                    if (AngularMomentumBoost) {
                        AddForceToGrid((int)i, SimCentre, 0.5f, (d, t) => -d, UseTangentialVector: true);
                        //AddForceToGrid((int)i, SimCentre, 0.5f, (d, t) => d);
                    } else if (SheerBoost) {
                        (int x, int y) = Grid1DUnfoldIndex(i);
                        GridMap.Velocity_GP[i].x += 0.25f * y;
                    }
                } else {
                    // if (Grid[i].Index.x == 0) {
                    //     //Apply vertical force to LHS wall.
                    //     Grid[i].Velocity.y += Settings.MouseRadius;
                    // }
                }
            });
            if (k == 2) {
                if (AngularMomentumBoost) AngularMomentumBoost = false;
                if (SheerBoost) SheerBoost = false;
            }


            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("External Forces: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
                SimulationTimer.Restart();
            }


            //Enforce Dirichlet Boundary Conditions - consider all neighbours of solid cells, and remove component facing into those cells.
            foreach (int Index in SolidCellLookup) {
                (int x, int y) = Grid1DUnfoldIndex(Index);
                if (x > 0 && GridMap.Velocity_GP[Index - 1].x > 0) {
                    //Stops LHS cell from having positive rightward velocity.
                    GridMap.Velocity_GP[Index - 1].x = 0;
                }
                if (y > 0 && GridMap.Velocity_GP[Index - GridWidth].y > 0) {
                    //Stops bottom cell from having positive upward velocity.
                    GridMap.Velocity_GP[Index - GridWidth].y = 0;
                }
                //Stops RHS and top cell from having negative velocity components, pointing inside the solid cell.
                if (GridMap.Velocity_GP[Index].x < 0) GridMap.Velocity_GP[Index].x = 0;
                if (GridMap.Velocity_GP[Index].y < 0) GridMap.Velocity_GP[Index].y = 0;
            }


            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("Boundary Conditions: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
                SimulationTimer.Restart();
            }


            //Projects fluid along its divergence-free component.
            if (Settings.ProjectionStepSize > 0f) {
                PCG.SetNbhdCoeffValues();
                PCG.StiffnessCoefficient = Settings.Stiffness;
                PCG.CalculateDivergence();

                float[] PreviousPressureMap = PCG.GetPreviousPressureMap();
                PCG.PCG_ICT(PreviousPressureMap);
                PCG.AssignPressureValues();
            }   
            


            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("Grid Solver: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
                SimulationTimer.Restart();
            }


            //Interpolates the grid velocities to the particles.
            InterpolateGridToParticles();

            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("Interpolation 2: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
                SimulationTimer.Restart();
            }


            //Updates particle positions using RK2, and fixing boundary conditions.
            MaxVelocityThisStep = UpdateParticlePositions(k);


            if (TimeSimulation) {
                SimulationTimer.Stop();
                Debug.Log("Position Updater: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms.");
            }

            //Tells the GPU to copy the particle data to the render buffer before rendering its next frame.
            GPUReadyFlag = true;
        }

        SimulationTime += Settings.TimeStep;

        if (TrackAngularMomentum) {
                float TotalAngularMomentum = 0f;
                foreach (Particle p in ParticleCont) {
                    float2 PositionToCentre = p.pos - SimCentre;
                    float OrbitalP = PositionToCentre.x * p.velocity.y - PositionToCentre.y * p.velocity.x;
                    float SpinP = 0f;
                    if (Settings.TransferMethod == GlobalSettings.TransferMethodType.AffinePIC) {
                        SpinP = p.cOperator_y.x - p.cOperator_x.y;
                    } else if (Settings.TransferMethod == GlobalSettings.TransferMethodType.RigidPIC) {
                        SpinP = p.AngularMomentumLoss.x + p.AngularMomentumLoss.y;
                    }
                    SpinP *= CellSize * CellSize / 4f;
                    TotalAngularMomentum += OrbitalP + SpinP;
                }
                TotalAngularMomentum /= ParticleCont.Length;
                Debug.Log("Total Angular Momentum this frame: " + TotalAngularMomentum + "kgm^2s^-1 (per particle).");

                //Writes to a file.
                if (DataWriter != null) {
                    DataWriter.WriteLine($"{SimulationTime},{TotalAngularMomentum}");
                }
            }

        float NewDeltaTime = CellSize / MaxVelocityThisStep;
        if (NewDeltaTime < deltaTime) {
            if (Settings.UpdateTimeStepSafety) {
                if (CellSize / MaxVelocity > 1 / 500f) {
                    Settings.TimeStep = 0.75f * NewDeltaTime * 2; //Multiply by 0.75 for safety, 2 for RK2...
                    Debug.Log($"DeltaTime value is too high for stable simulation - overwriting to: 1/{math.round(1 / Settings.TimeStep)}");
                } else {
                    Settings.TimeStep = 0.75f * (1 / 600f) * 2;
                    Debug.Log($"DeltaTime value is too high for stable simulation - overwriting MINIMUM to: 1/{math.round(1 / Settings.TimeStep)}. Prepare for a crash lol...");
                }
                if (Settings.SyncTimeStepToSystem) Time.fixedDeltaTime = Settings.TimeStep;
            } else {
                //Debug.Log($"DeltaTime value is too high for stable simulation - consider a value of 1/{math.round(0.5/NewDeltaTime)}...");
            }
        }
        //Debug.Log(MaxVelocityThisStep);
        if (MaxVelocityThisStep > MaxVelocity) {
            MaxVelocity = MaxVelocityThisStep;
        } else {
            // We want the colour of the fluid to represent it's current state, whilst also not being too 'contrasty'. To help with this,
            // we scale MaxVelcity depending on how close it it to the velocity in this iteration (s.t numerical instability is quickly resolved).
            MaxVelocity -= math.min(math.pow((MaxVelocity - MaxVelocityThisStep) / 50f, 2) / 2, 0.5f * MaxVelocity);
        }
    }



    





    // Function which applies some force to a MAC staggered grid cell, using a distance function.
    private void AddForceToGrid(int CellIndex, Vector2 ForcePoint, float ForceMagnitude, Func<float, float, float> DistanceFunction, float DistanceThreshold = float.MaxValue, bool UseTangentialVector = false) {
        //Checks if the grid cell is at least close enough to run more specific distance checks.
        Vector2 GridCentre = new(GridMap.Index2D[CellIndex].x + 0.5f, GridMap.Index2D[CellIndex].y + 0.5f);
        float GridDistance = Vector2.Distance(ForcePoint, GridCentre);

        if (GridDistance - 0.5f < DistanceThreshold) {
            //Run more specific calculations, then applies the formula to apply a force to the grid.
            Vector2[] GridOfsetValues = new Vector2[] { new(0.5f, 0), new(0, 0.5f) };
            foreach (Vector2 Ofset in GridOfsetValues) {
                Vector2 GridValuePoint = GridCentre + Ofset;
                Vector2 MouseGridVector = ForcePoint - GridValuePoint;

                Vector2 GridVectorNormalised = math.normalize(MouseGridVector);
                GridDistance = math.length(MouseGridVector);
                if (GridDistance > 0.1f && GridDistance < DistanceThreshold) { // Safety of distance.
                    float ScalingFactor = ForceMagnitude * DistanceFunction(GridDistance, DistanceThreshold);
                    if (Ofset.x > 0) {
                        GridMap.Velocity_GP[CellIndex].x += ScalingFactor * (UseTangentialVector ? -GridVectorNormalised.y : GridVectorNormalised.x);
                    } else {
                        GridMap.Velocity_GP[CellIndex].y += ScalingFactor * (UseTangentialVector ? GridVectorNormalised.x : GridVectorNormalised.y);
                    }
                }
            }
        }
    }

    private (bool, bool) CheckGridCellInMouseRange(int CellIndex, Vector2 MousePos, float DistanceThreshold = float.MaxValue) {
        bool XInRange = false, YInRange = false;
        //Checks if the grid cell is at least close enough to run more specific distance checks.
        Vector2 GridCentre = new(GridMap.Index2D[CellIndex].x + 0.5f, GridMap.Index2D[CellIndex].y + 0.5f);
        float GridDistance = Vector2.Distance(MousePos, GridCentre);

        if (GridDistance - 0.5f < DistanceThreshold) {
            //Run more specific calculations, then applies the formula to apply a force to the grid.
            Vector2[] GridOfsetValues = new Vector2[] { new(0.5f, 0), new(0, 0.5f) };
            foreach (Vector2 Ofset in GridOfsetValues) {
                Vector2 GridValuePoint = GridCentre + Ofset;
                Vector2 MouseGridVector = MousePos - GridValuePoint;

                Vector2 GridVectorNormalised = math.normalize(MouseGridVector);
                GridDistance = math.length(MouseGridVector);
                bool InRange = GridDistance > 0.1f && GridDistance < DistanceThreshold;
                if (Ofset.x == 0f) {
                    YInRange = InRange;
                } else {
                    XInRange = InRange;
                }
            }
        }
        return (XInRange, YInRange);
    }



    // Function which uses the new particle velocities to update their positions, using Runge-Kutta. We also handle collisions with walls
    // in this method, and thus boundary conditions.
    private float UpdateParticlePositions(int RK2Iteration, float CollisionEpsilon = 1e-10f) {
        float MaxVelocityThisStep = 0f;
        Parallel.For(0, ParticleCont.Length, i => {
            float2 ChangeInPosition;
            if (RK2Iteration == 1) { //Half-way through - update particle to intermediate position, then run simulation again.
                ChangeInPosition = ParticleCont[i].velocity * deltaTime;
                ParticleCont[i].pos += ChangeInPosition;
            } else {
                //Performs RK2 integration, using the two sample points.
                ChangeInPosition = ParticleCont[i].velocity * (2 * deltaTime);
                ParticleCont[i].pos = ParticleCont[i].pos_buffer + ChangeInPosition;

                //Calculates stabledt.
                MaxVelocityThisStep = math.max(MaxVelocityThisStep, math.length(ParticleCont[i].velocity));
            }

            // If the simulation is in a _very_ unstable state, it may be that the particle jumps across the solid border. In this case, we detect
            // if particles are outside the Grid bounds.
            //? Hmmm, this really does seem like a lot of unnecessary computation... Are we sure this is really needed for most uses? *Probably...*
            int NewGridPosX = (int)(ParticleCont[i].pos.x / CellSize);
            int NewGridPosY = (int)(ParticleCont[i].pos.y / CellSize);
            bool ParticleInBounds = (NewGridPosX >= 0) && (NewGridPosX < GridWidth) && (NewGridPosY >= 0) && (NewGridPosY < GridHeight);
            bool ParticleInSolid = !ParticleInBounds || GridMap.Type[Grid2DFlattenIndex(NewGridPosX, NewGridPosY)] == 0;
            if (!ParticleInSolid) {
                // If the particle is incredibly close to colliding with the solid, we will likely get numerical errors when interpolating through cells
                // (cannot access solid cell - fluid cell gets tiny to no contribution, leading to division by tiny numbers).
                float NewDeltaGridX = ParticleCont[i].pos.x % CellSize;
                float NewDeltaGridY = ParticleCont[i].pos.y % CellSize;
                float CollisionDistanceThreshold = 1e-10f;
                if (NewDeltaGridX < CollisionDistanceThreshold || NewDeltaGridY < CollisionDistanceThreshold) {
                    // Checks if there is a solid cell at (x-1, y-1).
                    if (GridMap.Type[Grid2DFlattenIndex(NewGridPosX - 1, NewGridPosY - 1)] == 0) { ParticleInSolid = true; }
                }
                if ((CellSize - NewDeltaGridX) < CollisionDistanceThreshold || (CellSize - NewDeltaGridY) < CollisionDistanceThreshold) {
                    // Checks if there is a solid cell at (x+1, y+1).
                    ParticleInSolid = GridMap.Type[Grid2DFlattenIndex(NewGridPosX + 1, NewGridPosY + 1)] == 0;
                }
            }
            if (ParticleInSolid) {
                // The most recent iteration pushed the particle into a wall - push it back inside.
                // Finds the point of intersection where the particle hit the wall, then moves half a cell in the normal direction of this solid cell.
                float2 p_old = ParticleCont[i].pos - ChangeInPosition;

                // Finds the closest vertical cell boundary intersection point from p_old to p_new.
                float2 VerticalBoundaryPosition;
                bool VerticlePointsRightward = ChangeInPosition.x > 0;
                VerticalBoundaryPosition.x = CellSize * (ParticleCont[i].GridIndex.x + (VerticlePointsRightward ? 1 : 0));
                // Lambda represents the time for the particle to hit this specific spot - used to determine which boundary is closer.
                float lambdaVertical = math.abs(ChangeInPosition.x) < CollisionEpsilon ? 1e30f : (VerticalBoundaryPosition.x - p_old.x) / ChangeInPosition.x;

                // Finds the closest horizontal cell boundary intersection point from p_old to p_new.
                float2 HorizontalBoundaryPosition;
                bool HorizontalPointsUpwards = ChangeInPosition.y > 0;
                HorizontalBoundaryPosition.y = CellSize * (ParticleCont[i].GridIndex.y + (HorizontalPointsUpwards ? 1 : 0));
                float lambdaHorizontal = (math.abs(ChangeInPosition.y) < CollisionEpsilon) ? 1e30f : (HorizontalBoundaryPosition.y - p_old.y) / ChangeInPosition.y;

                // Gets the closest boundary point, then moves the particle normal 0.1*CellSize (into boundary layer).
                if (lambdaVertical < lambdaHorizontal) {
                    // Vertical wall is closer in the particle's trajectory.
                    VerticalBoundaryPosition.y = p_old.y + lambdaVertical * ChangeInPosition.y;
                    ParticleCont[i].pos = VerticalBoundaryPosition + new float2(0.1f * CellSize, 0) * (VerticlePointsRightward ? -1f : 1f);
                    //Debug.Log("Caught " + i + " Vertically: " + lambdaVertical + " " + lambdaHorizontal + " at cell: " + NewGridPosX + " " + NewGridPosY);
                } else {
                    // Horizontal wall is closer in the particle's trajectory.
                    HorizontalBoundaryPosition.x = p_old.x + lambdaHorizontal * ChangeInPosition.x;
                    ParticleCont[i].pos = HorizontalBoundaryPosition + new float2(0, 0.1f * CellSize) * (HorizontalPointsUpwards ? -1f : 1f);
                    //Debug.Log("Caught " + i + " Horizontally: " + lambdaVertical + " " + lambdaHorizontal + " at cell: " + NewGridPosX + " " + NewGridPosY);
                }
                ParticleCont[i].velocity = float2.zero; // Enforces boundary conditions again + the no-slip boundary condition.

            }
            ParticleCont[i].SetGridPosVars(CellSize); // Calibrates GridIndex and DeltaGrid for the new position.

            // ParticleInBounds = (ParticleCont[i].GridIndex.x >= 0) && (ParticleCont[i].GridIndex.x < GridWidth) && (ParticleCont[i].GridIndex.y >= 0) && (ParticleCont[i].GridIndex.y < GridHeight);
            // if (!ParticleInBounds || Grid2DAccess(ParticleCont[i].GridIndex.x, ParticleCont[i].GridIndex.y).Type == 0) {
            //     Debug.Log("oh poo: " + i + " " + ParticleInBounds);
            // }

        });
        return MaxVelocityThisStep;
    }
    

}
