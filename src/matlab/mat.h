#ifndef MATLAB_MAT_H
#define MATLAB_MAT_H

#include <iostream>
#include "matrix.h"


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

    enum controllers {
      SHOOTER,
      DRIVE,
      NUM_CONTROLLERS
    };

    int num_inputs;
    int num_outputs;
    int num_states;

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
    ss_controller(int inputs, int outputs, int states, controllers controller);

    void reset();
    ~ss_controller();

    /**
      * The heart of the state space controller
      * @param outmat the output matrix of left and right
      * voltages
      * @param r the r matrix, which holds the goal left
      * and right positions and velocities
      * @param y the current left and right distances
      */
    void update( matrix *R, struct matrix *Y);

};
#endif // MATLAB_MAT_H
