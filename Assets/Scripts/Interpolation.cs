using Unity.Mathematics;
using System.Threading;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.VisualScripting;
using System.Runtime.CompilerServices;
using UnityEngine;
using System.Linq;

public partial class FluidSim {

    //Functions to Bilinearly Interpolate between all the particles, and all the cells in the grid, using a weighted average.
    //This also handles the classification of cells (ie: setting them as either fluid or air cells).
    public void InterpolateParticlesToGrid() {
        InitGrid(false); //Resets all fluid cells to the 'air' type.

        //We assume that there are no particles stuck in solids, and set the grid cell type accordingly. Note that Type = 1 represents fluid.
        for (int i = 0; i < ParticleCont.Length; i++) { //Unfortunately, we cannot parallelise this, as cells need to be in the right order for triangular matrices :).
            int CellIndex = Grid2DFlattenIndex(ParticleCont[i].GridIndex.x, ParticleCont[i].GridIndex.y);
            if (GridMap.Type[CellIndex] == 2) {
                GridMap.Type[CellIndex] = 1;
                //As this cell is now a fluid cell, we add this to a list, so that we can easily calculate its neighbours for the CG method.
                PCG.AddCell(CellIndex);
            }
        }

        Parallel.For(0, ParticleCont.Length, i => {

            float Inertia = 0f;
            if (Settings.TransferMethod == GlobalSettings.TransferMethodType.RigidPIC) {
                for (int n = 1; n <= 2; n++) {
                    var (index, delta) = ParticleCont[i].GetStaggeredGridPosVars(n == 1, n == 2, CellSize); // Note that we are negating delta, for better use in affine calculations.
                    float w1 = (1 + delta.x) * (1 + delta.y);
                    float w2 = -(1 + delta.x) * delta.y;
                    float w3 = delta.x * delta.y;
                    float w4 = -delta.x * (1 + delta.y);
                    Inertia += CalculateRigidInertia(delta * CellSize, new float4(w1, w4, w2, w3));
                }
            }

            //Calculates signed distance weights from the particle's location to each of its neighbouring x grid cell.
            //Note that if the grid is a solid, we immediately set its weight to 0, meaning the solid cell cannot influence the
            //particle's motion (we cannot access said velocities).
            for (int n = 1; n <= 2; n++) {
                //a : 1 = x component, 2 = y component.
                var (index, delta) = ParticleCont[i].GetStaggeredGridPosVars(n == 1, n == 2, CellSize); // Note that we are negating delta, for better use in affine calculations.

                //Calculate the weights to each of the 4 neighbouring velocity components, //Calculate the weights to each of the 4 neighbouring velocity components, ensuring we zero out any solid cell contributions that arise from user forces and/or projection from the previous step.
                int GridIndex = index.y * GridWidth + index.x;
                float w1 = (1 + delta.x) * (1 + delta.y);
                float w2 = -(1 + delta.x) * delta.y;
                float w3 = delta.x * delta.y;
                float w4 = -delta.x * (1 + delta.y);

                float2 c = float2.zero;
                if (Settings.TransferMethod == GlobalSettings.TransferMethodType.AffinePIC) {
                    c = n == 1 ? ParticleCont[i].cOperator_x : ParticleCont[i].cOperator_y;
                } else if (Settings.TransferMethod == GlobalSettings.TransferMethodType.RigidPIC) {
                    if (Inertia > 1e-10f) {
                        float TotalL = ParticleCont[i].AngularMomentumLoss.x + ParticleCont[i].AngularMomentumLoss.y;
                        if (n == 1) {
                            c.y = -TotalL / Inertia;
                        } else {
                            c.x = TotalL / Inertia;
                        }
                    }
                }
                float cDotd = math.dot(c, delta) * CellSize;

                if (GridMap.Type[GridIndex] == 0) w1 = 0;
                if (GridMap.Type[GridIndex + GridWidth] == 0) w2 = 0;
                if (GridMap.Type[GridIndex + GridWidth + 1] == 0) w3 = 0;
                if (GridMap.Type[GridIndex + 1] == 0) w4 = 0;

                //Updates the 4 neighbouring grid cell velocity values, and total weights.
                if (n == 1) {
                    //Updates x component.
                    float vComponent = ParticleCont[i].velocity.x;
                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex].x, w1 * (vComponent + cDotd));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex].x, w1);

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex + 1].x, w4 * (vComponent + cDotd + (c.x * CellSize)));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex + 1].x, w4);

                    GridIndex += GridWidth;
                    cDotd += c.y * CellSize;

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex].x, w2  * (vComponent + cDotd));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex].x, w2);

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex + 1].x, w3 * (vComponent + cDotd + (c.x * CellSize)));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex + 1].x, w3);
                } else {
                    //Updates y component.
                    float vComponent = ParticleCont[i].velocity.y;
                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex].y, w1 * (vComponent + cDotd));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex].y, w1);

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex + 1].y, w4 * (vComponent + cDotd + (c.x * CellSize)));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex + 1].y, w4);

                    GridIndex += GridWidth;
                    cDotd += c.y * CellSize;

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex].y, w2 * (vComponent + cDotd));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex].y, w2);

                    RaceConditionAdd(ref GridMap.Velocity_PG[GridIndex + 1].y, w3 * (vComponent + cDotd + (c.x * CellSize)));
                    RaceConditionAdd(ref GridMap.TotalWeight[GridIndex + 1].y, w3);
                }

            }

            // Density interpolation (weighted from the centre of each cell, hence staggering by both pos).
            if (Settings.Stiffness > 0) {
                var (index, delta) = ParticleCont[i].GetStaggeredGridPosVars(true, true, CellSize);
                int GridIndex = index.y * GridWidth + index.x;
                if (GridMap.Type[GridIndex] == 1) {
                    float w1 = (1 + delta.x) * (1 + delta.y);
                    RaceConditionAdd(ref GridMap.Density[GridIndex], w1);
                }
                if (GridMap.Type[GridIndex + 1] == 1) {
                    float w4 = -(1 + delta.x) * delta.y;
                    RaceConditionAdd(ref GridMap.Density[GridIndex + 1], w4);
                }
                GridIndex += GridWidth;
                if (GridMap.Type[GridIndex] == 1) {
                    float w2 = delta.x * delta.y;
                    RaceConditionAdd(ref GridMap.Density[GridIndex], w2);
                }
                if (GridMap.Type[GridIndex + 1] == 1) {
                    float w3 = -delta.x * (1 + delta.y);
                    RaceConditionAdd(ref GridMap.Density[GridIndex + 1], w3);
                }
            }
        });

        //Calibrates the weighted sums for each of the velocity components (noting that the border of the cells will always be solid).
        Parallel.For(0, GridMap.Length(), n => {
            // Only calibrate non-solid cells.
            if (GridMap.Type[n] != 0) {
                GridMap.CalibrateWeights((int)n);
                GridMap.CopyToBuffer((int)n);
            }
        });
    }


    // Note: alfie does not like this function. Adds Value to dir, and saves in dir, whilst handling race conditions.
    public static void RaceConditionAdd(ref float dir, float Value) {
        float CurrentVal, AttemptVal;
        do { // Constantly attempts to append Value to dir, not doing so only if it is locked.
            CurrentVal = dir;
            AttemptVal = CurrentVal + Value;
        } while (CurrentVal != Interlocked.CompareExchange(ref dir, AttemptVal, CurrentVal));
    }





    public void InterpolateGridToParticles() {
        Parallel.For(0, ParticleCont.Length, i => {

            //Calculates signed distance weights from the particle's location to each of its neighbouring x grid cell.
            //Note that if the grid is a solid, we immediately set its weight to 0, meaning the solid cell cannot influence the
            //particle's motion (we cannot access said velocities).
            for (int n = 1; n <= 2; n++) {
                //a : 1 = x component, 2 = y component.
                var (index, delta) = ParticleCont[i].GetStaggeredGridPosVars(n == 1, n == 2, CellSize);
                int GridIndex = index.y * GridWidth + index.x;

                //Calculate the weights to each of the 4 neighbouring velocity components (by assumption, we have zeroed out any solid cell contributions that arise from user forces and/or projection).
                float w1 = (1 + delta.x) * (1 + delta.y);
                float w2 = -(1 + delta.x) * delta.y;
                float w3 = delta.x * delta.y;
                float w4 = -delta.x * (1 + delta.y);
                float TotalWeight = w1 + w2 + w3 + w4;

                if (Settings.TransferMethod == GlobalSettings.TransferMethodType.AffinePIC) {
                    // We have a boundary condition error in APIC, in which (when a fluid particle reaches the right or top boundary), a sudden shift
                    // in grid velocities cause a massive action of angular momentum conservation towards the top right corner of the simulation.
                    // Therefore, in the top row and rightward column of fuid cells, we resort to PIC, rather than APIC.
                    if (n == 1) {
                        ParticleCont[i].cOperator_x = CalculateWeightGradient(GridIndex, delta, true);
                        if (APICBoundarySafety && GridMap.Type[ParticleCont[i].GridIndex.x + 1] == 0) ParticleCont[i].cOperator_x.x = 0;
                    } else {
                        ParticleCont[i].cOperator_y = CalculateWeightGradient(GridIndex, delta, false);
                        if (APICBoundarySafety && GridMap.Type[ParticleCont[i].GridIndex.x + GridWidth] == 0) ParticleCont[i].cOperator_y.y = 0;
                    }
                } else if (Settings.TransferMethod == GlobalSettings.TransferMethodType.RigidPIC) {
                    float4 w = new(w1, w4, w2, w3);
                    if (n == 1) {
                        ParticleCont[i].AngularMomentumLoss.x = CalculateRigidMomentumLoss(GridIndex, delta * CellSize, w, true);
                    } else {
                        ParticleCont[i].AngularMomentumLoss.y = CalculateRigidMomentumLoss(GridIndex, delta * CellSize, w, false);
                    }
                }

                //Updates particle velocity based on these weights.
                float NewVelocityLerp = 0;
                float FLIPWeight = Settings.PICFLIPRatio; //1 means purely FLIP, 0 means purely PIC.
                
                if (n == 1) {
                    if (FLIPWeight < 1) { //Adds PIC component.
                        NewVelocityLerp = (1 - FLIPWeight) * (w1 * GridMap.Velocity_GP[GridIndex].x + w2 * GridMap.Velocity_GP[GridIndex + GridWidth].x
                        + w3 * GridMap.Velocity_GP[GridIndex + GridWidth + 1].x + w4 * GridMap.Velocity_GP[GridIndex + 1].x) / TotalWeight;
                    }
                    if (FLIPWeight > 0) { //Adds FLIP component.
                        NewVelocityLerp += FLIPWeight * (ParticleCont[i].velocity.x + (w1 * GridMap.GetVelocityChange(GridIndex).x + w2 * GridMap.GetVelocityChange(GridIndex + GridWidth).x
                        + w3 * GridMap.GetVelocityChange(GridIndex + GridWidth + 1).x + w4 * GridMap.GetVelocityChange(GridIndex + 1).x) / TotalWeight);
                    }
                    ParticleCont[i].velocity.x = NewVelocityLerp;
                } else {
                    if (FLIPWeight < 1) { //Adds PIC component.
                        NewVelocityLerp = (1 - FLIPWeight) * (w1 * GridMap.Velocity_GP[GridIndex].y + w2 * GridMap.Velocity_GP[GridIndex + GridWidth].y
                        + w3 * GridMap.Velocity_GP[GridIndex + GridWidth + 1].y + w4 * GridMap.Velocity_GP[GridIndex + 1].y) / TotalWeight;
                    }
                    if (FLIPWeight > 0) { //Adds FLIP component.
                        NewVelocityLerp += FLIPWeight * (ParticleCont[i].velocity.y + (w1 * GridMap.GetVelocityChange(GridIndex).y + w2 * GridMap.GetVelocityChange(GridIndex + GridWidth).y
                        + w3 * GridMap.GetVelocityChange(GridIndex + GridWidth + 1).y + w4 * GridMap.GetVelocityChange(GridIndex + 1).y) / TotalWeight);
                    }
                    ParticleCont[i].velocity.y = NewVelocityLerp;
                }

            }
        });

    }




    // GridIndex is the start Grid Position to get velocity components from. d represents the saturated distance from the particle
    // to the bottom left cell face.
    private float CalculateRigidMomentumLoss(int GridIndex, float2 d, float4 w, bool HorizontalFace) { //d represents the signed distance from the particle to that grid index.
        // Carries out the calculation "sum w (x_i - x_p) cross u_i".
        float4 UnweightedMomentum;
        if (HorizontalFace) {
            UnweightedMomentum = new(GridMap.Velocity_GP[GridIndex].x, GridMap.Velocity_GP[GridIndex + 1].x, GridMap.Velocity_GP[GridIndex + GridWidth].x, GridMap.Velocity_GP[GridIndex + GridWidth + 1].x);
            UnweightedMomentum.xy *= d.y;
            UnweightedMomentum.zw *= d.y + CellSize;
            return -math.dot(w, UnweightedMomentum);
        } else {
            UnweightedMomentum = new(GridMap.Velocity_GP[GridIndex].y, GridMap.Velocity_GP[GridIndex + 1].y, GridMap.Velocity_GP[GridIndex + GridWidth].y, GridMap.Velocity_GP[GridIndex + GridWidth + 1].y);
            UnweightedMomentum.xz *= d.x;
            UnweightedMomentum.yw *= d.x + CellSize;
            return math.dot(w, UnweightedMomentum);
        }
    }
    private float CalculateRigidInertia(float2 d, float4 w) { //d represents the signed distance from the particle to that grid index.
        // Carries out the operation w[i] * x[i] dot x[i], where x = {d, d + new float2(CellSize, 0), d + new float2(0, CellSize), d + new float2(CellSize, CellSize)};
        float d2 = math.dot(d,d);
        float c2 = CellSize * CellSize;
        float dx2c = 2 * d.x * CellSize + c2;
        float dy2c = 2 * d.y * CellSize + c2;
        float Inertia = w.x * d2 + w.y * (d2 + dx2c) + w.z * (d2 + dy2c) + w.w * (d2 + dx2c + dy2c);
        return Inertia;
    }


    private float2 CalculateWeightGradient(int GridIndex, float2 d, bool HorizontalFace) { //d represents the signed distance from the particle to that grid index.
        // Carries out the operation c = nabla w u.x or c = nabla w u.y.
        // Note that, from d.x, d.y being in range [-1,1], and the first grid cell always being to the bottom-left of the particle, we can always determine its sign as -.
        float absdx = 1 + d.x; // absdx1 = 1 - math.abs(d.x + 1) = -d.x;
        float absdy = 1 + d.y; // absdy1 = 1 - math.abs(d.y + 1) = -d.y;

        float4 c_x = new(-absdy, absdy, d.y, -d.y);
        float4 c_y = new(-absdx, d.x, absdx, -d.x);
        float4 GridVelocities;
        if (HorizontalFace) {
            GridVelocities = new(GridMap.Velocity_GP[GridIndex].x, GridMap.Velocity_GP[GridIndex + 1].x, GridMap.Velocity_GP[GridIndex + GridWidth].x, GridMap.Velocity_GP[GridIndex + GridWidth + 1].x);
        } else {
            GridVelocities = new(GridMap.Velocity_GP[GridIndex].y, GridMap.Velocity_GP[GridIndex + 1].y, GridMap.Velocity_GP[GridIndex + GridWidth].y, GridMap.Velocity_GP[GridIndex + GridWidth + 1].y);
        }
                
        return new float2(math.dot(c_x, GridVelocities), math.dot(c_y, GridVelocities)) / CellSize; //! should be -??
    }





    private float2 CalculateWeightGradientPaper(int GridIndex, float2 d, float4 w, bool HorizontalFace) { //d represents the signed distance from the particle to that grid index.
        d *= CellSize;
        float4 vCrossd;
        if (HorizontalFace) {
            vCrossd = new(GridMap.Velocity_GP[GridIndex].x, GridMap.Velocity_GP[GridIndex + 1].x, GridMap.Velocity_GP[GridIndex + GridWidth].x, GridMap.Velocity_GP[GridIndex + GridWidth + 1].x);
            vCrossd *= d.y;
            return new float2(math.dot(w, vCrossd), 0f);
        } else {
            vCrossd = new(GridMap.Velocity_GP[GridIndex].y, GridMap.Velocity_GP[GridIndex + 1].y, GridMap.Velocity_GP[GridIndex + GridWidth].y, GridMap.Velocity_GP[GridIndex + GridWidth + 1].y);
            vCrossd *= d.x;
            return new float2(0f, math.dot(w, vCrossd));
        }
    }

    private float2 CalculateWeightGradientPaper2(int GridIndex, float2 d, float4 w, bool HorizontalFace) { //d represents the signed distance from the particle to that grid index.
        d *= CellSize;
        float4 vel;
        float2 c;
        if (HorizontalFace) {
            vel = new(GridMap.Velocity_GP[GridIndex].x, GridMap.Velocity_GP[GridIndex + 1].x, GridMap.Velocity_GP[GridIndex + GridWidth].x, GridMap.Velocity_GP[GridIndex + GridWidth + 1].x);
            float4 vel_x = vel;
            vel_x.xz *= d.x;
            vel_x.yw *= d.x + CellSize;
            float4 vel_y = vel;
            vel_y.xy *= d.y;
            vel_y.zw *= d.y + CellSize;
            c = new float2(math.dot(w, vel_x), math.dot(w, vel_y));
        } else {
            vel = new(GridMap.Velocity_GP[GridIndex].y, GridMap.Velocity_GP[GridIndex + 1].y, GridMap.Velocity_GP[GridIndex + GridWidth].y, GridMap.Velocity_GP[GridIndex + GridWidth + 1].y);
            float4 vel_x = vel;
            vel_x.xz *= d.x;
            vel_x.yw *= d.x + CellSize;
            float4 vel_y = vel;
            vel_y.xy *= d.y;
            vel_y.zw *= d.y + CellSize;
            c = new float2(math.dot(w, vel_x), math.dot(w, vel_y));
        }
        return c * (4.0f / (CellSize * CellSize));
    }
    
}
