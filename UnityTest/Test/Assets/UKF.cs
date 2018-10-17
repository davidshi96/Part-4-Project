using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System;
using UnityEngine;

public class UKF : MonoBehaviour {

    // Number of States
    private int L;

    // Measurements number
    private int m;

    // The alpha coefficient, characterize sigma-points dispersion around mean
    private double alpha;

    // The ki
    private double ki;

    // The beta coefficient, characterize type of distribution (2 for normal one) 
    private double beta;

    // Scale factor
    private double lambda;

    // Scale factor
    private double c;

    // Means weights
    private Matrix<double> Wm;

    // Covariance weights
    private Matrix<double> Wc;
    
	// State
    private Matrix<double> x;

    // Covariance
    private Matrix<double> P;

	// Std of process
    private double q;

	// Std of measurement
    private double r;

	// Covariance of process
    private Matrix<double> Q;

	// Covariance of measurement 
    private Matrix<double> R;

    // Constructor of Unscented Kalman Filter
    public UKF()
    {
        L = 3;
        m = 3;
        q = 0.05;
        r = 0.3;

        x = q * Matrix.Build.Random(L, 1); //initial state with noise
        P = Matrix.Build.Diagonal(L, L, 1); //initial state covraiance

        Q = Matrix.Build.Diagonal(L, L, q * q); //covariance of process
        R = Matrix.Build.Dense(m, m, r * r); //covariance of measurement  

        alpha = 0.01f;
        ki = 0;
        beta = 2f;
        lambda = alpha * alpha * (L + ki) - L;
        c = L + lambda;

        //weights for means for the sigma points
        //0.5/c is the weight for all other mean and covariances.
        Wm = Matrix.Build.Dense(1, (2 * L + 1), 0.5 / c);
        Wm[0, 0] = lambda / c;

        //weights for covariance for the sigma points
        Wc = Matrix.Build.Dense(1, (2 * L + 1));
        Wm.CopyTo(Wc);
        Wc[0, 0] = Wm[0, 0] + 1 - alpha * alpha + beta;

        c = Math.Sqrt(c);
    }

    public void UpdateFilter(Vector3 position, Vector3 Velocity, Vector3 Acceleration, float deltaT)
    {
        //z is 3 rows 1 column
        var z = Matrix.Build.Dense(m, 1, 0);
        z[0, 0] = position.x;
        z[1, 0] = position.y;
        z[2, 0] = position.z;

        Vector3 V = Velocity;
        Vector3 A = Acceleration;
        float T = deltaT;
        //sigma points around x
        Matrix<double> X = GetSigmaPoints(x, P, c);


        //unscented transformation of process
        // X1=sigmas(x1,P1,c) - sigma points around x1
        //X2=X1-x1(:,ones(1,size(X1,2))) - deviation of X1
        Matrix<double>[] ut_f_matrices = UnscentedTransform(X, V, A, T, Wm, Wc, L, Q);
        Matrix<double> x1 = ut_f_matrices[0];
        Matrix<double> X1 = ut_f_matrices[1];
        Matrix<double> P1 = ut_f_matrices[2];
        Matrix<double> X2 = ut_f_matrices[3];

        //unscented transformation of measurments
        Matrix<double>[] ut_h_matrices = UnscentedTransform(X, V, A, T, Wm, Wc, L, Q);
        Matrix<double> z1 = ut_h_matrices[0];
        Matrix<double> Z1 = ut_h_matrices[1];
        Matrix<double> P2 = ut_h_matrices[2];
        Matrix<double> Z2 = ut_h_matrices[3];

        //transformed cross-covariance
        Matrix<double> P12 = (X2.Multiply(Matrix.Build.Diagonal(Wc.Row(0).ToArray()))).Multiply(Z2.Transpose());

        Matrix<double> K = P12.Multiply(P2.Inverse());

        //state update
        x = x1.Add(K.Multiply(z.Subtract(z1)));
        //covariance update 
        P = P1.Subtract(K.Multiply(P12.Transpose()));
    }

    public double[] getState()
    {
        return x.ToColumnArrays()[0];
    }

    public double[,] getCovariance()
    {
        return P.ToArray();
    }

    // Unscented Transformation: returns the transformed mean, transformed sampling points, transformed covariance, and transformed deviations
    private Matrix<double>[] UnscentedTransform(Matrix<double> X,
        Vector3 V, Vector3 A, float T, Matrix<double> Wm, Matrix<double> Wc, int n, Matrix<double> R)
    {

        int L = X.ColumnCount;
        Matrix<double> y = Matrix.Build.Dense(n, 1, 0);
        Matrix<double> Y = Matrix.Build.Dense(n, L, 0);

        Matrix<double> row_in_X;
        Vector3 position;
        Vector3 NextPosition;
        for (int k = 0; k < L; k++)
        {
            position.x = (float)X[0, k];
            position.y = (float)X[1, k];
            position.z = (float)X[2, k];
            //need to the pass each row into the function f
            NextPosition = position + T * V + 0.5f * (A - Physics.gravity) * T * T;
            //sets rows_in_x to be equal to each sigma point 
            row_in_X = Matrix.Build.Dense(m, 1, 0);
            if (NextPosition.y < 0)
            {
                row_in_X[0, 0] = position.x;
                row_in_X[1, 0] = position.y;
                row_in_X[2, 0] = position.z;
            }
            else
            {
                row_in_X[0, 0] = NextPosition.x;
                row_in_X[1, 0] = NextPosition.y;
                row_in_X[2, 0] = NextPosition.z;
            }
            
            Y.SetSubMatrix(0, Y.RowCount, k, 1, row_in_X);
            y = y.Add(Y.SubMatrix(0, Y.RowCount, k, 1).Multiply(Wm[0, k]));
        }

        Matrix<double> Y1 = Y.Subtract(y.Multiply(Matrix.Build.Dense(1, L, 1)));
        Matrix<double> P = Y1.Multiply(Matrix.Build.Diagonal(Wc.Row(0).ToArray()));
        P = P.Multiply(Y1.Transpose());
        P = P.Add(R);

        Matrix<double>[] output = { y, Y, P, Y1 };
        return output;
    }

    // Returns sigma points around reference point
    private Matrix<double> GetSigmaPoints(Matrix<double> x, Matrix<double> P, double c)
    {
        // Cholesky decomposition is alternative to square root
        Matrix<double> A = P.Cholesky().Factor;

        A = A.Multiply(c);
        A = A.Transpose();

        int n = x.RowCount;

        Matrix<double> Y = Matrix.Build.Dense(n, n, 1);
        for (int j = 0; j < n; j++)
        {
            Y.SetSubMatrix(0, n, j, 1, x);
        }

        Matrix<double> X = Matrix.Build.Dense(n, (2 * n + 1));
        X.SetSubMatrix(0, n, 0, 1, x);

        Matrix<double> Y_plus_A = Y.Add(A);
        X.SetSubMatrix(0, n, 1, n, Y_plus_A);

        Matrix<double> Y_minus_A = Y.Subtract(A);
        X.SetSubMatrix(0, n, n + 1, n, Y_minus_A);

        return X;
        
    }

    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
