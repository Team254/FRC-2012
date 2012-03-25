#include <iostream>
#include "matrix.h"

const int num_inputs = 1;
const int num_outputs = 1;
const int num_states = 2;

//this is some complex code written by Austin Schuh which was made obsolete
//by the use of a different matrix library, but was kept because it
//represents a lot of work and could possibly be recycled in the future

/*

MatrixXd s2z_A(MatrixXd A, MatrixXd B, double dt)
{
	ComplexEigenSolver<MatrixXd> eigensolver(A);

	Matrix<std::complex<double>, num_states, num_states> diag(num_states, num_states);
	Matrix<std::complex<double>, num_states, num_states> diage(num_states, num_states);
	ComplexEigenSolver<MatrixXd>::EigenvalueType eval = eigensolver.eigenvalues();
	for (int j = 0; j < num_states; j ++) {
		for (int i = 0; i < num_states; i ++) {
			diag(i, j) = 0.0;
			diage(i, j) = 0.0;
		}
	}
	for (int i = 0; i < num_states; i ++) {
		std::complex<double> lambda = eval(i, 0);
		diag(i, i) = std::exp(lambda * dt);
		if (std::abs(lambda) < 1.0e-14) {
			diag(i, i) = dt;
		} else {
			diag(i, i) = (std::exp(lambda * dt) - 1.0) / lambda;
		}
	}

	Matrix<std::complex<double>, num_states, num_states> dmy = (eigensolver.eigenvectors() * diag * eigensolver.eigenvectors().inverse());
	MatrixXd ans(num_states, num_states);
	for (int j = 0; j < num_states; j ++) {
		for (int i = 0; i < num_states; i ++) {
			ans(i, j) = std::real(dmy(i, j));
		}
	}
	return ans;
}
MatrixXd s2z_B(MatrixXd A, MatrixXd B, double dt)
{
	ComplexEigenSolver<MatrixXd> eigensolver(A);

	Matrix<std::complex<double>, num_states, num_states> diag(num_states, num_states);
	Matrix<std::complex<double>, num_states, num_states> diage(num_states, num_states);
	ComplexEigenSolver<MatrixXd>::EigenvalueType eval = eigensolver.eigenvalues();
	for (int j = 0; j < num_states; j ++) {
		for (int i = 0; i < num_states; i ++) {
			diag(i, j) = 0.0;
			diage(i, j) = 0.0;
		}
	}
	for (int i = 0; i < num_states; i ++) {
		std::complex<double> lambda = eval(i, 0);
		diag(i, i) = std::exp(lambda * dt);
		if (std::abs(lambda) < 1.0e-14) {
			diag(i, i) = dt;
		} else {
			diag(i, i) = (std::exp(lambda * dt) - 1.0) / lambda;
		}
	}

	Matrix<std::complex<double>, num_states, num_states> dmy = (eigensolver.eigenvectors() * diage * eigensolver.eigenvectors().inverse());
	MatrixXd ans(num_states, num_states);
	for (int j = 0; j < num_states; j ++) {
		for (int i = 0; i < num_states; i ++) {
			ans(i, j) = std::real(dmy(i, j));
		}
	}
	return ans;
}
*/

/**
  * A state space controller
  * After being set up properly, this controller works as such:
  * Given two matrices:
  *     r: the robot's goal position and velocity
  *     y: the robot's current left and right encoder values
  * The controller will return a left and right voltage (with 12V being the
  * maximum voltage) to make the robot behave as well as possible
  */
class ss_controller
{
	public:
	    const int num_inputs;
	    const int num_outputs;
	    const int num_states;

	    //the state matrices, calculated and imported from matlab
	    struct matrix *A;
	    struct matrix *B;
	    struct matrix *C;
	    struct matrix *D;
	    struct matrix *L;
	    struct matrix *K;
            // other state matrices
            struct matrix *X;
            struct matrix *X_hat;
            struct matrix *U;
            struct matrix *U_max;
            struct matrix *U_min;
            struct matrix *b_u;
            struct matrix *l_y;
            struct matrix *l_c;
            struct matrix *a_lc;
            struct matrix *alc_xhat;
            struct matrix *xhatp1;

	    //temporary matrices needed because of the C-style matrix lib
            struct matrix *U_tmp;
	public:
		ss_controller() :
			num_inputs(1),
			num_outputs(1),
			num_states(2)
		{
			//initalizes all the matrices
			A = init_matrix(num_states, num_states);
			B = init_matrix(num_states, num_outputs);
			C = init_matrix(num_outputs, num_states);
			D = init_matrix(num_outputs, num_outputs);
			L = init_matrix(num_states, num_outputs);
			K = init_matrix(num_outputs, num_states);
			X = init_matrix(num_states, 1);
			X_hat = init_matrix(num_states, 1);
			U = init_matrix(num_outputs, 1);
			U_max = init_matrix(num_outputs, 1);
			U_min = init_matrix(num_outputs, 1);
			U_tmp = init_matrix(num_states, 1);
                        b_u = init_matrix(num_states, 1);
                        l_y = init_matrix(num_states, 1);
                        l_c = init_matrix(num_states, num_states);
                        a_lc = init_matrix(num_states, num_states);
                        alc_xhat = init_matrix(num_states, 1);
                        xhatp1 = init_matrix(num_states, 1);

			//import the matlab-computed matrix values
			#include "controller.h"

			//A_precomp = A - (L * C) - (B * K);
			//Br_precomp = B * K;
		}
		
		void reset()
		{

		}
		~ss_controller() {
		}

		/**
		  * The heart of the state space controller
		  * @param outmat the output matrix of left and right
		  * voltages
		  * @param r the r matrix, which holds the goal left
		  * and right positions and velocities
		  * @param y the current left and right distances
		  */
		void update(struct matrix *outmat, struct matrix *R, struct matrix *Y)
		{
			//printf("r: %f %f\n", R->data[0], R->data[1]);
			printf("xhat: %f %f\n", X_hat->data[0], X_hat->data[1]);
			//U = K * (R - X_hat);
                        matrix_minus(U_tmp, R, X_hat);
                        matrix_mult(U, K, U_tmp);

                        /*
			// If U is outside the hardware range, limit it before it 
			// gets added to the observer.
			for (int i = 0; i < number_of_outputs; i ++) {
				if (U[i] > U_max[i]) {
					U[i] = U_max[i];
				} else if (U[i] < U_min[i]) {
					U[i] = U_min[i];
				}
			}
                        */
                        for(int i=0; i < num_outputs; i++) {
                            double* u_i = U->data+i;
                            double u_max = U_max->data[i];
                            double u_min = U_min->data[i];
                            if(*u_i > u_max) {
                                *u_i = u_max;
                            } else if (*u_i < u_min) {
                                *u_i = u_min;
                            }
                        }
                        // X_hat = (A - L * C) * X_hat + L * Y + B * U;
                        matrix_mult(b_u, B, U);
                        matrix_mult(l_y, L, Y);
                        matrix_mult(l_c, L, C);
                        matrix_minus(a_lc, A, l_c);
                        matrix_mult(alc_xhat, a_lc, X_hat);
                        matrix_add(xhatp1, alc_xhat, l_y);
                        matrix_add(X_hat, xhatp1, b_u);

                    /*
			//(ebakan): C-style computations = FFFFFFFFFFFFFUUUUUUU
			//MatrixXd nxhat = (A_precomp * xhat) + (Br_precomp * r) + (L * y);

			//compute xhat stuff
			matrix_mult(xhat_a, A_precomp, xhat);
			matrix_mult(xhat_b, Br_precomp, r);
			matrix_mult(xhat_l, L, y);

			matrix_add(xhat,xhat_a,xhat_b);
			matrix_add(xhat,xhat,xhat_l);

			//return K * r - K * xhat;

			matrix_mult(outmat_r, K, r);
			matrix_mult(outmat_xhat, K, xhat);
			matrix_minus(outmat, outmat_r, outmat_xhat);
                        */

		}
		
};


/*
int main()
{
	MatrixXd y(2, 1);
	MatrixXd r(4, 1);
	ss_controller ssc;

	y << 1.0, 1.0;
	r << 0.0, 0.0, 0.0, 0.0;

	std::cout << ssc.update(r, y) << std::endl;
}
*/

