using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using System.Runtime.CompilerServices;

//Class holding all the data used by the preconditioned conjugate gradient method, including the stored matrices.
//As these matrices are very sparse (holding only information regarding a cell's 4 immediate neighbours).
//we implement these via the CSR format, holding only the non-zero entries????
public class PreconditionedConjugateGradient {
    VectorMethods Vector = new VectorMethods();
    Grid GridMap;
    private int[] FluidCells; //Represents the data of all fluids cells at the current timestep. As the matrix A (containing the neighbourhood coefficients)
    //is very sparse, and is not needed for many calculations, construct A's values on the fly using data from this array, and use these in calculations.
    public float[] WeightedDivergence; //Divergence of each cell, multiplied by CellSize^2.
    public float[] Pressure;
    private int Size;
    public float StepSize;

    private float Rho0 = 4f;
    public float StiffnessCoefficient = 1;


    private int GridWidth;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Grid2DFlattenIndex(int x, int y) => y * GridWidth + x;


    public PreconditionedConjugateGradient(int MaxSize, float CellSize, ref Grid GridMap, int GridWidth) {
        this.GridMap = GridMap;
        FluidCells = new int[MaxSize];
        this.GridWidth = GridWidth;
        Clear(CellSize);
    }

    public void Clear(float NewCellSize) {
        Size = 0;
        StepSize = NewCellSize;
    }

    public void AddCell(int CellIndex) {
        FluidCells[Size] = CellIndex;
        GridMap.IndexInFluidCells[CellIndex] = Size; // Creates dictionary lookup.
        Size++;
        GridMap.Neighbours[CellIndex, 0] = 0; // Clears neighbours.
    }
    public int GetSize() {
        return Size;
    }
    public void SetTargetDensity(float Density) {
        Rho0 = Density;
    }


    // Function to set and retrieve the values in the matrix A, containing matrix coefficient values: 1 if the cell is a fluid neighbour cell, 0 if the cells are not neighbnours,
    // and -{number of fluid neighbours} if i=j. This is to allow the finite differential operator to function.
    public void SetNbhdCoeffValues() {
        for (int i = 0; i < Size; i++) {
            // Loops over all neighbours using the cell's IndexInFluidCells lookup table.
            int FluidCellIndex = FluidCells[i];
            int[] GridNeighbours = new int[] { FluidCellIndex - 1, FluidCellIndex + 1, FluidCellIndex - GridWidth, FluidCellIndex + GridWidth };

            foreach (int NeighbourIndex in GridNeighbours) {
                if (GridMap.Type[NeighbourIndex] == 1) { // Fluid cell. Makes fluid cell NeighbourIndex the neighbour of fluid cell i
                    //? It's quite annoying I can't also make i the neighbour of NeighbourIndex... Is this possible?
                    GridMap.Neighbours[FluidCellIndex, 0]++; // Each cell has one more neighbour.
                    GridMap.Neighbours[FluidCellIndex, GridMap.Neighbours[FluidCellIndex, 0]] = GridMap.IndexInFluidCells[NeighbourIndex];
                }
            }
        }
    }
    public int GetNbhdCoeffValue(int i, int j) {
        if (i == j) {
            // Adds the non-solid neighbours to the fluid cell count. Must be positive for discrete differential operators to function (make positive definite).
            return GridMap.NonSolidNeighbourCount[FluidCells[i]]; 
        } else {
            for (int n = 1; n <= GridMap.Neighbours[FluidCells[i], 0]; n++) {
                if (j == GridMap.Neighbours[FluidCells[i], n]) { return -1; }
            }
            return 0; // j could not be found in the list of neighbours.
        }
    }


    public void CalculateDivergence() {
        // Instantiated Divergence Array.
        WeightedDivergence = new float[Size];
        for (int i = 0; i < Size; i++) {
            int n = FluidCells[i];
            float Divergence = GridMap.Velocity_GP[n].x + GridMap.Velocity_GP[n].y;
            if (GridMap.Type[n - 1] != 0) { Divergence -= GridMap.Velocity_GP[n - 1].x; }
            if (GridMap.Type[n - GridWidth] != 0) { Divergence -= GridMap.Velocity_GP[n - GridWidth].y; }
            WeightedDivergence[i] = -StepSize * (Divergence - StiffnessCoefficient * (GridMap.Density[n] - Rho0)); // We make this negative as A is forced to be positive definite. Ap = d === -Ap = -d.

            // float normdiff = (Grid[n].Density - Rho0) / Rho0;
            // WeightedDivergence[i] = -StepSize * (Divergence - StiffnessCoefficient * (normdiff / (normdiff + 1)));
        }
    }


    // Retrieves the pressure values stored in the cells, from the previous fluid iteration, then stores this as a good
    // initial guess for the next PCG_ICT iteration.
    public float[] GetPreviousPressureMap() {
        float[] PrevPressure = new float[Size];
        for (int i = 0; i < Size; i++) {
            PrevPressure[i] = GridMap.Pressure[FluidCells[i]];
        }
        return PrevPressure;
    }




    public void PCG_ICT(float[] InitGuess = null) {
        // System.Diagnostics.Stopwatch tim = new System.Diagnostics.Stopwatch();
        // tim.Start();

        int NoIterations = 0; // Represents k.
        float Threshold = 2e-4f;
        InitGuess ??= new float[Size]; // Default to array of 0s. Note InitGuess represents p (will constantly iterate).
        // tim.Stop();
        // Debug.Log("1) " + tim.Elapsed.TotalMilliseconds);
        // tim.Restart();


        // Computes the Matrix M = KK', an approximation of A, where K is a lower-triangular and K' is K's transpose.
        CSRTglMatrix K = ComputeICTMatrix();
        //K.OutputToConsole();

        // tim.Stop();
        // Debug.Log("2) " + tim.Elapsed.TotalMilliseconds);
        // tim.Restart();

        float[] Ax = new float[Size];
        NbhdCoeffVectorMultiplication(InitGuess, ref Ax);
        float[] r = Vector.Subtract(WeightedDivergence, Ax);
        if (Vector.NormInf(r) < Threshold) { Pressure = InitGuess; return; }
        float[] z = new float[Size];
        float[] IntermediateStorage = new float[Size];

        // tim.Stop();
        // Debug.Log("3) " + tim.Elapsed.TotalMilliseconds);
        // tim.Restart();

        z = SolveICT(K, r, ref IntermediateStorage);

        // tim.Stop();
        // Debug.Log("4) " + tim.Elapsed.TotalMilliseconds);
        // tim.Restart();

        float[] x = Vector.Copy(z);
        float r_dot_old, r_dot_new, alpha;
        r_dot_new = Vector.Dot(r, z);

        // tim.Stop();
        // Debug.Log("5) " + tim.Elapsed.TotalMilliseconds);
        // tim.Restart();

        while (NoIterations < 1000) {
            r_dot_old = r_dot_new;
            NbhdCoeffVectorMultiplication(x, ref Ax);

            // if (NoIterations == 0) {
            //     tim.Stop();
            //     Debug.Log("6) " + tim.Elapsed.TotalMilliseconds);
            //     tim.Restart();
            // }

            alpha = r_dot_old / Vector.Dot(x, Ax);
            VarNextGeneration(ref InitGuess, alpha, x);
            VarNextGeneration(ref r, -alpha, Ax);

            // if (NoIterations == 0) {
            //     tim.Stop();
            //     Debug.Log("7) " + tim.Elapsed.TotalMilliseconds);
            //     tim.Restart();
            // }
            
            if (Vector.NormInf(r) < Threshold) { Pressure = InitGuess; return; } //Debug.Log($"Converged in {NoIterations} Iterations");
            z = SolveICT(K, r, ref IntermediateStorage);

            // if (NoIterations == 0) {
            //     tim.Stop();
            //     Debug.Log("8) " + tim.Elapsed.TotalMilliseconds);
            //     tim.Restart();
            // }

            r_dot_new = Vector.Dot(r, z);
            PrepareNextVector(z, r_dot_new / r_dot_old, ref x);
            //Vector.OutputToConsole(InitGuess);
            NoIterations++;
            
            // if (NoIterations == 1) {
            //     tim.Stop();
            //     Debug.Log("9) " + tim.Elapsed.TotalMilliseconds);
            //     tim.Restart();
            // }
        }
        Debug.Log($"Error: Maximum Iteration Count Exceeded... Residual: {Vector.NormInf(r)}");
        Pressure = InitGuess;
    }

    // Multiplies the Neighbourhood Coefficient Matrix, A, by some vector v, by looping over A's non-zero entries (ie: the cell corresponding to each
    //specific row's neighbours) and performing matrix-vector multiplication. 
    private void NbhdCoeffVectorMultiplication(float[] v, ref float[] p) {
        for (int i = 0; i < Vector.Dimension(v); i++) {
            // Loops through all of i's neighbours (ie: non-zero values in the i'th row of A) and multiplies the required values.
            int FluidCellIndex = FluidCells[i];
            p[i] = GridMap.NonSolidNeighbourCount[FluidCellIndex] * v[i]; // corresponding to A[i,i]. 
            for (int n = 1; n <= GridMap.Neighbours[FluidCellIndex, 0]; n++) {
                p[i] -= v[GridMap.Neighbours[FluidCellIndex, n]]; // Specifically, 1 * v[n]: A[i,n] = -1 if n is i's neighbour, and 0 else.
            }
        }
    }

    private void VarNextGeneration(ref float[] u, float Scalar, float[] v) {
        for (int i = 0; i < Vector.Dimension(u); i++) {
            u[i] = u[i] + Scalar * v[i];
        }
    }
    private void PrepareNextVector(float[] v, float Scalar, ref float[] u) {
        for (int i = 0; i < Vector.Dimension(u); i++) {
            u[i] = v[i] + Scalar * u[i];
        }
    }


    // Computes the lower triangular matrix K, using Incomplete Cholesky Drop-Tolerance Factorisaion.
    private CSRTglMatrix ComputeICTMatrix(float tau = 1e-3f) {
        CSRTglMatrix K = new CSRTglMatrix(Size, 5 * Size); // Each row has a maximum of 5 non-zero entries (the cell, and its 4 neighbours).
        for (int i = 0; i < Size; i++) {
            // We have the condition of K[i,j] = 0 if A[i,j] = 0. Thus, we loop over all of i's neighbours, then itself (ie: A[i]'s non-zero entries),
            // noting that the diagonal element (ie: A[i,i]) will be the last value in the row stream.
            float w;
            int FluidCellIndex = FluidCells[i];
            for (int n = 1; n <= GridMap.Neighbours[FluidCellIndex, 0]; n++) {
                int j = GridMap.Neighbours[FluidCellIndex, n];
                if (j < i) { //Lower Triangular Check.
                    w = (-1 - K.GetNbhdValueApproximation(i, j)) / K.GetFinalValueInRow(j); //-1 comes from A[i,j] = -1 if j is a neighbour of i.
                    if (math.abs(w) > tau) { K.AddIncrementalEntry(w, i, j); } // Drop-Tolerance - preserve sparsity if value is negligable.
                }
            }
            // Applies diagonal value.
            w = GridMap.NonSolidNeighbourCount[FluidCellIndex] - K.GetRowSquareSum(i); // FluidCells[i].NonSolidNeighbourCount = A[i,i].
            w = math.max(w, 1e-10f); // Safety against sqrt-ing negative numbers.
            K.AddIncrementalEntry(math.sqrt(w), i, i);

            K.FinishRowEntryStream(i);
        }
        return K;
    }

    // Solves the linear system Mz = r via M = KK' approximation, and solving Ka = r for a, then K'z = a for z via forward & backward substitution respectively.
    // Note that K is a lower-triangular matrix - to form the upper triangular matrix K', we transpose GetValue(i,j) -> GetValue(j,i).
    // Also, due to the 'triangular' composition of z in the Backwards solver, a's values are only accessed once, and so can be passed by reference to save space.
    private float[] SolveICT(CSRTglMatrix K, float[] r, ref float[] a) {
        // Clears intermediate storage, a.
        Array.Clear(a, 0, Size);
        // Solves Ka = r for a, via forward substitution.
        K.ForwardSubstitutionSolver(r, ref a);
        // Solves K'z = a for z, via backward substitution, and saves the result into a.
        K.BackwardsSubstitutionSolver(ref a);
        return a;
    }


    // Calculates the change in velocity calculated by this pressure gradient, done by the discrete gradient operator, and assuming a water density of 1.
    // Note that pressure values are stored at the centres of each face, via the definition of the laplace operator, so calculating the gradient operator
    // on this puts us back to our MAC values.
    public void AssignPressureValues() {
        for (int i = 0; i < Size; i++) {
            // Finds the cell directly above and rightward of the cell in question. If this is a solid or air cell, we won't be able to find it
            // in the lookup table, and thus give it a pressure value of 0. Also assumes a solid border around the simulation (can safely access Grid[id.x + 1, id.y + 1]).
            int FluidCellIndex = FluidCells[i];

            // Assigns the pressure stored in the most recent iteration to the cell, so it can be retrieved for future simulations.
            GridMap.Pressure[FluidCellIndex] = Pressure[i];

            // Attempts to retrive the neighbour to this cell's right & top, and add pressure gradients if found.
            int NeighbourIndex = GridMap.IndexInFluidCells[FluidCellIndex + 1];
            // If our fluid cell has solid neighbours, we need to explicitly check that the neighbour is not a solid. If it is, completely neglect velocity update (dp = 0).
            if (GridMap.NonSolidNeighbourCount[FluidCellIndex] == 4 || GridMap.Type[FluidCellIndex + 1] != 0) {
                // Update velocity based on pressure gradient, using p[air] = 0.
                GridMap.Velocity_GP[FluidCellIndex].x += Pressure[i] / StepSize;
                if (NeighbourIndex >= 0) { // The cell is a fluid cell - update pressure gradient.
                    GridMap.Velocity_GP[FluidCellIndex].x -= Pressure[NeighbourIndex] / StepSize;
                }
            }

            NeighbourIndex = GridMap.IndexInFluidCells[FluidCellIndex + GridWidth];
            if (GridMap.NonSolidNeighbourCount[FluidCellIndex] == 4 || GridMap.Type[FluidCellIndex + GridWidth] != 0) {
                GridMap.Velocity_GP[FluidCellIndex].y += Pressure[i] / StepSize;
                if (NeighbourIndex >= 0) { // The cell is a fluid cell - update pressure gradient.
                    GridMap.Velocity_GP[FluidCellIndex].y -= Pressure[NeighbourIndex] / StepSize;
                }
            }
        }
    }


    // Outputs the full coefficient matrix and divengence values to the debug window.
    public void OutputDebugInfo() {
        string OutputString = "<color=green>Indexes:</color> [";
        for (int i = 0; i < Size; i++) {
            OutputString += GridMap.Index2D[FluidCells[i]] + " ";         
        }
        Debug.Log(OutputString + "]");

        OutputString = "<color=cyan>Matrix:</color>\n";
        for (int i = 0; i < Size; i++) {
            OutputString += "[";
            for (int j = 0; j < Size; j++) {
                OutputString += GetNbhdCoeffValue(i,j).ToString("F2").PadRight(6);
            }
            OutputString += "]\n";
        }
        
        Debug.LogFormat("{0}", OutputString);
        Debug.Log($"<color=yellow>Weighted Divergence:</color> {string.Join(", ", WeightedDivergence)}");
    }

}