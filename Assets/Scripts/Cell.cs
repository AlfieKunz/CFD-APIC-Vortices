using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class Grid {

    public int Size;
    public int2 GridDimensions;

    public int2[] Index2D; //Which cell are we referring to?
    public int[] Type; //Symbolises which type each cell is in the grid: 0=solid, 1=fluid, 2=air.

    // The two arrays hold the staggered (MAC) grid velocities, s.t the velocity components are located at the right / top face of each cell.
    // PG refers to the velocity field immediately upon transfer to the grid, and GP is the divergent-free, modified field that is interpolated back to particles.
    public float2[] Velocity_PG; 
    public float2[] Velocity_GP;
    public float[] Density;

    public float[] DistanceSortValues; //Denotes an ordering of non-solid cells, for determining where to spawn fluid first.

    public float[] Pressure;

    public int[] IndexInFluidCells; //Produces a dictionary-like lookup for grid cells, to quickly build up neighbourhood coeff values. -1 denotes a non-fluid cell.
    public int[,] Neighbours; //Denotes the indices of the cell's fluid neighbours in the PCG model (flattened array containing only fluid cells).
    //Index 0 of this array refers to the number of fluid neighbours.
    public int[] NonSolidNeighbourCount;

    public float2[] TotalWeight;

    public Grid(int Size, int2 GridDimensions) {
        this.Size = Size;
        this.GridDimensions = GridDimensions;

        Index2D = new int2[Size];
        Type = new int[Size];
        Velocity_PG = new float2[Size];
        Velocity_GP = new float2[Size];
        Density = new float[Size];
        DistanceSortValues = new float[Size];
        Pressure = new float[Size];
        IndexInFluidCells = new int[Size];
        Neighbours = new int[Size, 5];
        NonSolidNeighbourCount = new int[Size];
        TotalWeight = new float2[Size];
    }
    public int Length() {
        return Size;
    }


    public int GetFlatIndex(int x, int y) {
        return y * GridDimensions.x + x;
    }

    public void InitCell(int x, int y) {
        int FlatIndex = GetFlatIndex(x, y);
        Index2D[FlatIndex] = new int2(x, y);
        Init(FlatIndex);
    }
    public void InitCell(int Index) {
        Index2D[Index] = new int2(Index % GridDimensions.x, Index / GridDimensions.x);
        Init(Index);
    }
    private void Init(int Index) {
        Type[Index] = 2;
        for (int i = 0; i < 5; i++) {
            Neighbours[Index, i] = 0;
        }
        NonSolidNeighbourCount[Index] = 4;
        ResetCell(Index);
    }

    public void ResetCell(int Index) {
        IndexInFluidCells[Index] = -1;
        //NonSolidNeighbourCount = 4;
        //Neighbours[0] = 0;
        if (Type[Index] == 1) { Type[Index] = 2; }
        Velocity_PG[Index] = float2.zero;
        TotalWeight[Index] = float2.zero;
        Density[Index] = 0f;
    }

    public void CalibrateWeights(int Index) {
        if (TotalWeight[Index].x > 0) {
            Velocity_PG[Index].x /= TotalWeight[Index].x;
        }
        if (TotalWeight[Index].y > 0) {
            Velocity_PG[Index].y /= TotalWeight[Index].y;
        }
    }

    public void CopyToBuffer(int Index) {
        Velocity_GP[Index] = Velocity_PG[Index];
    }
    public float2 GetVelocityChange(int Index) {
        return Velocity_GP[Index] - Velocity_PG[Index];
    }
}