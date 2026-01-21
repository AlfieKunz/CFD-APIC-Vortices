using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

//Structure holding all the information of a particle in the simulation. Note we use 3D variables here, even though it is a 2D simulation (shader likes it more, for some reason...)
public struct Particle {
    public float2 pos;

    public float2 pos_buffer; //Represents the position of the particle at the start of each simulation step (as opposited to pos, which changes multiple times
                              //due to having to sample the velocity at multiple points due to RK2).
                              //Position of the particle relative to the grid (GridIndex), and within its specific cell (deltaGrid).
    public float2 velocity;
    public int2 GridIndex;
    //Because of the definition of the staggered grid, we also need to stagger this value by CallSize / 2,
    //to make it easier to calculate the distance from the particle to some specific grid velocity component.
    public float2 DeltaGrid;

    // The two vectors below construct the Omega^\dagger operator for APIC transfers, and represents the angular momentum information
    // of the particle. We construct these during interpolation from grid to particle, for use in the next transfer to grid.
    public float2 cOperator_x;
    public float2 cOperator_y;

    public float2 AngularMomentumLoss;

    //Method for Settting the two above variables.
    public void SetGridPosVars(float CellSize) {
        int GridPosX = (int)(pos.x / CellSize);
        int GridPosY = (int)(pos.y / CellSize);
        GridIndex = new int2(GridPosX, GridPosY);
        DeltaGrid = new float2(pos.x % CellSize, pos.y % CellSize);
    }


    public (int2, float2) GetStaggeredGridPosVars(bool StaggerVx, bool StaggerVy, float CellSize) {
        int2 GridIndexStagger = new(GridIndex.x - 1, GridIndex.y - 1);
        float2 DeltaGridStagger = new(DeltaGrid.x / CellSize, DeltaGrid.y / CellSize);
        if (StaggerVx) {
            //The x-field is moved up from the cell corners by CellSize / 2 - stagger.
            if (DeltaGridStagger.y > 0.5f) {
                GridIndexStagger.y += 1;
                DeltaGridStagger.y -= 0.5f;
            } else {
                DeltaGridStagger.y += 0.5f;
            }
        } 
        if (StaggerVy) {
            //The y-field is moved rightwards from the cell corners by CellSize / 2 - stagger.
            if (DeltaGridStagger.x > 0.5f) {
                GridIndexStagger.x += 1;
                DeltaGridStagger.x -= 0.5f;
            } else {
                DeltaGridStagger.x += 0.5f;
            }
        }
        return (GridIndexStagger, -DeltaGridStagger);
    }

    public void CopyToBuffer() {
        pos_buffer = pos;
    }
}



public struct ParticleRender {
    public float2 pos;
    public float2 velocity;

    public void Init(ref Particle p) {
        pos = p.pos;
        velocity = p.velocity;
    }

}
