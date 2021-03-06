#include "matlab/mat.h"

#include <iostream>

#include "matlab/matrix.h"

ss_controller::ss_controller(int inputs, int outputs, int states, controllers controller) :
  num_inputs(inputs),
  num_outputs(outputs),
  num_states(states)
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
  switch (controller) {
    case SHOOTER:
      #include "shootercontroller.h"
      break;
    case DRIVE:
      #include "drivecontroller.h"
      break;
    default:
      break;
  }
}

void ss_controller::reset() {
}

ss_controller::~ss_controller() {
}

void ss_controller::update( matrix *R, struct matrix *Y)
{
  matrix_minus(U_tmp, R, X_hat);
  matrix_mult(U, K, U_tmp);

  for(int i=0; i < num_outputs; i++) {
    double* u_i = U->data+i;
    double u_max = U_max->data[i];
    double u_min = U_min->data[i];
    if (*u_i > u_max) {
      *u_i = u_max;
    } else if (*u_i < u_min) {
      *u_i = u_min;
    }
  }

  matrix_mult(b_u, B, U);
  matrix_mult(l_y, L, Y);
  matrix_mult(l_c, L, C);
  matrix_minus(a_lc, A, l_c);
  matrix_mult(alc_xhat, a_lc, X_hat);
  matrix_add(xhatp1, alc_xhat, l_y);
  matrix_add(X_hat, xhatp1, b_u);

}


