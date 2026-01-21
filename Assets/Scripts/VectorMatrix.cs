using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Mathematics;
using System.Drawing;
using System.Linq;

public class VectorMethods {
    public int Dimension(float[] Vector) { return Vector.Length; }
    public float Norm2(float[] Vector) { return MathF.Sqrt(Dot(Vector, Vector)); }
    public float NormInf(float[] Vector) {
        float MaxValue = Vector[0];
        for (int i = 1; i < Dimension(Vector); i++) {
            MaxValue = math.max(MaxValue, Vector[i]);
        }
        return MaxValue;
     }

    // We assume that v1 and v2 have the same dimension.
    public float[] Add(float[] v1, float[] v2) {
        float[] result = new float[Dimension(v1)];
        for (int i = 0; i < Dimension(v1); i++) {
            result[i] = v1[i] + v2[i];
        }
        return result;
    }
    public float[] Subtract(float[] v1, float[] v2) {
        float[] result = new float[Dimension(v1)];
        for (int i = 0; i < Dimension(v1); i++) {
            result[i] = v1[i] - v2[i];
        }
        return result;
    }

    public float[] Multiply(float[] v1, float Scalar) {
        float[] result = new float[Dimension(v1)];
        for (int i = 0; i < Dimension(v1); i++) {
            result[i] = v1[i] * Scalar;
        }
        return result;
    }

    public float Dot(float[] v1, float[] v2) {
        float result = 0f;
        for (int i = 0; i < Dimension(v1); i++) {
            result += v1[i] * v2[i];
        }
        return result;
    }

    public void Copy(float[] v_old, ref float[] v_new) {
        for (int i = 0; i < Dimension(v_old); i++) {
            v_new[i] = v_old[i];
        }
    }
    public float[] Copy(float[] v) {
        float[] result = new float[Dimension(v)];
        for (int i = 0; i < Dimension(v); i++) {
            result[i] = v[i];
        }
        return result;
    }

    // Checks if two vectors are the same, by cross-comparing every index.
    // Tau denotes how different must each vector component be to be classified as 'different' (numerical error).
    public bool Compare(float[] u, float[] v, float tau = 1e-6f) {
        if (Dimension(u) != Dimension(v)) { return false; }
        for (int i = 0; i < Dimension(u); i++) {
            if (math.abs(u[i] - v[i]) > tau) { return false; }
        }
        return true;
    }

    public void OutputToConsole(float[] v) {
        Debug.Log("{" + string.Join(", ", v) + "}");
    }
}






// Class storing sparse, square, triangular matrices in my simulation using the Compressed-Sparse-Row format.
// In my pCGM methods, we assume said matrices are symmetric and positive definite.
public class CSRTglMatrix {
    private float[] Values; // The non-zero values stored in the matrix.
    private int[] ColIndices; // The column index of each value in the matrix (synced with Values).
    private int[] RowPointers; // For which index in Values[] does each row start with?
    private int Size, ValuesAdded;

    // Constructor methods.
    public CSRTglMatrix(int Length, int NonZeroCount) { Init(Length, NonZeroCount); }
    public void Init(int Length, int NonZeroCount) {
        Values = new float[NonZeroCount];
        ColIndices = new int[NonZeroCount];
        RowPointers = new int[Length + 1]; // We often loop RowPtr[i] to RowPtr[i+1] - RowPtr[Size+1] contains NonZeroCount.
        Size = Length;
        ValuesAdded = 0;
    }

    // Overloads for converting a matrix (2d array) to a sparse CSR matrix.
    public CSRTglMatrix(float[,] Matrix, int NonZeroCount) : this(Matrix.GetLength(0), NonZeroCount) {
        float Value;
        for (int i = 0; i < Size; i++) {
            for (int j = 0; j < Size; j++) {
                // Adds each non-zero value to the matrix, in incremental order.
                Value = Matrix[i, j];
                if (Value != 0f) { AddIncrementalEntry(Value, i, j); }
            }
            // At the end of the row, update the row pointer. 
            FinishRowEntryStream(i);
        }
    }

    public CSRTglMatrix(float[,] Matrix) : this(Matrix, CalculateNonZeroCount(Matrix)) { }
    // Calculates how many non-zero values are in the matrix, for another overload.
    private static int CalculateNonZeroCount(float[,] Matrix) {
        int NonZeroCount = 0;
        int Length = Matrix.GetLength(0);
        for (int i = 0; i < Length; i++) {
            for (int j = 0; j < Length; j++) {
                if (Matrix[i, j] != 0f) { NonZeroCount++; }
            }
        }
        return NonZeroCount;
    }

    // Overload for copying one CSR matrix to another.
    public CSRTglMatrix(CSRTglMatrix SourceMatrix) {
        Size = SourceMatrix.Size;
        ValuesAdded = SourceMatrix.ValuesAdded;

        Values = new float[SourceMatrix.Values.Length];
        Array.Copy(SourceMatrix.Values, Values, SourceMatrix.Values.Length);

        ColIndices = new int[SourceMatrix.ColIndices.Length];
        Array.Copy(SourceMatrix.ColIndices, ColIndices, SourceMatrix.ColIndices.Length);

        RowPointers = new int[Size + 1];
        Array.Copy(SourceMatrix.RowPointers, RowPointers, Size + 1);
    }

    // Resets the matrix.
    public void Clear() { Init(Size, Values.Length); }



    // Methods for adding new values to the matrix. We assume that values are added in order (filling each row
    // incrementally, L->R), s.t we can construct RowPointers as we go (note that Values & ColIndices require ordering).
    // The second method sets the RowPointer for a row, once all values have been entered.
    public void AddIncrementalEntry(float Value, int RowIndex, int ColIndex) {
        Values[ValuesAdded] = Value;
        ColIndices[ValuesAdded] = ColIndex;
        ValuesAdded++;
    }
    public void FinishRowEntryStream(int RowIndex) {
        // Note that the first value inputted will always be the first in its row; RowPointers[0 -> RowIndex] = 0;
        RowPointers[RowIndex + 1] = ValuesAdded;
    }

    // Gets a specific value from the matrix.
    public float GetValue(int RowIndex, int ColIndex) {
        //if (RowIndex == ColIndex) { return GetFinalValueInRow(RowIndex); } // As M is lower-triangular.
        int RowStart = RowPointers[RowIndex];
        int RowEnd = RowPointers[RowIndex + 1] - 1;
        // Uses binary search to find the item, using ColIndex == ColIndices[i] (noting that, for a specific row,
        // all items are in ascending order of column).
        while (RowStart <= RowEnd) {
            int MidValue = RowStart + (RowEnd - RowStart) / 2;
            if (ColIndices[MidValue] == ColIndex) {
                return Values[MidValue];
            } else if (ColIndices[MidValue] < ColIndex) {
                RowStart = MidValue + 1;
            } else {
                RowEnd = MidValue - 1;
            }
        }
        return 0f; // No value could be retrived - assume the user attempted to access a null value.
    }

    // Retrieves the first and last value that is stored in some row of the matrix. As M is lower triangular, we can infer that GetFinalValueInRow(i) 
    // = M(i,i) (ie: its diagonal value), noting that this is non-zero as the NBHDCoeff Matrix will always be a number 2-4.
    public float GetFirstValueInRow(int RowIndex) {
        int RowStartIndex = RowPointers[RowIndex];
        return Values[RowStartIndex];
    }
    public float GetFinalValueInRow(int RowIndex) {
        int RowEndIndex = RowPointers[RowIndex + 1] - 1;
        return Values[RowEndIndex];
    }

    public void OutputDebugInfo() {
        Debug.Log("Values:         {" + string.Join(", ", Values) + "}");
        Debug.Log("Column Indices: {" + string.Join(", ", ColIndices) + "}");
        Debug.Log("Row Pointers:   {" + string.Join(", ", RowPointers) + "}");
    }


    // Gets the entire array referring to a specific row in the matrix (useful for building a 2d array from the CSR).
    public float[] GetRow(int RowIndex) {
        float[] RowValues = new float[Size];
        int RowStart = RowPointers[RowIndex];
        int RowEnd = RowPointers[RowIndex + 1];
        for (int i = RowStart; i < RowEnd; i++) {
            // Copies non-zero entries to the specific column.
            RowValues[ColIndices[i]] = Values[i];
        }
        return RowValues;
    }

    // Returns the sum: M[NoRows,0] + M[NoRows,1] + ... + M[NoRows, NoRows - 1].
    public float GetRowSquareSum(int RowIndex) {
        float Diag = 0;
        int RowStart = RowPointers[RowIndex];
        int RowEnd = RowPointers[RowIndex + 1] - 1; // Stops before the last diagonal.
        for (int i = RowStart; i < RowEnd; i++) {
            Diag += Values[i] * Values[i];
        }
        return Diag;
    }
    public float GetNbhdValueApproximation(int i, int j) {
        float Value = 0;
        int PointerI = RowPointers[i];
        int RowEndI = RowPointers[i + 1];

        int PointerJ = RowPointers[j];
        int RowEndJ = RowPointers[j + 1];
        while (PointerI < RowEndI && PointerJ < RowEndJ) {
            int ColI = ColIndices[PointerI];
            int ColJ = ColIndices[PointerJ];
            if (ColI >= j || ColJ >= j) break; //j < i via triangular.

            if (ColI == ColJ) {
                Value += Values[PointerI] * Values[PointerJ];
                PointerI++;
                PointerJ++;
            } else if (ColI < ColJ) {
                PointerI++;
            } else {
                PointerJ++;
            }
        }
        return Value;
    }

    // Applies the operation Mv, where M is the CSR matrix and v is some vector of choise.
    public float[] VectorMultiply(float[] v) {
        float[] p = new float[Size];
        for (int RowIndex = 0; RowIndex < Size; RowIndex++) {
            int RowStart = RowPointers[RowIndex];
            int RowEnd = RowPointers[RowIndex + 1];
            for (int i = RowStart; i < RowEnd; i++) {
                // Gets all non-zero columns of CSR, finds the accompanying indexed value in v, then multiplies them to get the value in p.
                p[RowIndex] += Values[i] * v[ColIndices[i]];
            }
        }
        return p;
    }

    public void OutputToConsole() {
        string OutputString = "<color=cyan>Matrix:</color>\n";
        for (int i = 0; i < Size; i++) {
            OutputString += "[";
            for (int j = 0; j < Size; j++) {
                OutputString += GetValue(i, j).ToString("F2").PadRight(6);
            }
            OutputString += "]\n";
        }
        Debug.LogFormat("{0}", OutputString);
    }


    // Solves the linear system Ax = v for x via the process of Forward Substitution, exploiting the fact that M is lower triangular.
    // See https://en.wikipedia.org/wiki/Triangular_matrix#Forward_substitution for more info on the method's specifics.
    public void ForwardSubstitutionSolver(float[] v, ref float[] x) {
        x[0] = v[0] / GetFinalValueInRow(0); // M[0,0] via triangular.
        int RowStart, RowEnd = RowPointers[1];
        for (int i = 1; i < Size; i++) {
            float WeightedSum = 0;
            RowStart = RowEnd;
            RowEnd = RowPointers[i + 1];
            for (int n = RowStart; n < RowEnd; n++) {
                // Gets all non-zero columns of CSR, finds the accompanying indexed value in v, then multiplies them.
                WeightedSum += Values[n] * x[ColIndices[n]];
            }
            x[i] = (v[i] - WeightedSum) / GetFinalValueInRow(i); // M[i,i] via triangular.
        }
    }

    // The Backwards Substitution Solver involves lots of Matrix-vector multiplication on an upper-triangular matrix (namely M^T), which our row-based
    // CS format matrix is incredibly inefficient at. Instead of transposing K initially to get a CSC format matrix, we stick to the advantages of CSR,
    // calculating all components of M[i,j]v[j], then adding each weighted component to the j'th row of x, s.t these values in x can be used for future iterations.
    // This builds up our equations for x in a triangular fashion.
    public void BackwardsSubstitutionSolver(ref float[] x) {
        for (int i = 0; i < Size; i++) {
            x[i] = x[i] / GetFinalValueInRow(i); // Sets initial values.
        }
        int RowStart = RowPointers[Size], RowEnd;
        for (int i = Size - 1; i > 0; i--) {
            // Calculates the row start & end values in CSR form (ie: the i'th column in CSC), remembering to ignore diagonal elements (already initialised).
            RowEnd = RowStart - 1;
            RowStart = RowPointers[i];
            for (int n = RowStart; n < RowEnd; n++) {
                x[ColIndices[n]] -= Values[n] * x[i] / GetFinalValueInRow(ColIndices[n]); // Noting that we are taking transposed values of M.
            }
        }
    }

}
