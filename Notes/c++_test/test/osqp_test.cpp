/*
 * @Author: wangdezhao
 * @Date: 2022-04-14 01:10:20
 * @LastEditTime: 2022-04-14 15:00:46
 * @FilePath: /osqp_eigen/home/next/routing_planning/Notes/c++_test/test/osqp_test.cpp
 * @Copyright:
 */
#include "osqp/osqp.h"

#include <iostream>
#include <vector>

#include "stdio.h"

OSQPData *FormulateProblem() {
//    OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

    c_float P_x[4] = {
            4.00000000000000000000,
            1.00000000000000000000,
            1.00000000000000000000,
            2.00000000000000000000,
    };
    c_int P_nnz = 4;
    c_int P_i[4] = {
            0,
            1,
            0,
            1,
    };
    c_int P_p[3] = {
            0,
            2,
            4,
    };
    c_float q[2] = {
            1.00000000000000000000,
            1.00000000000000000000,
    };
    c_float A_x[4] = {
            1.00000000000000000000,
            1.00000000000000000000,
            1.00000000000000000000,
            1.00000000000000000000,
    };
    c_int A_nnz = 4;
    c_int A_i[4] = {
            0,
            1,
            0,
            2,
    };
    c_int A_p[3] = {
            0,
            2,
            4,
    };
    c_float l[3] = {
            1.00000000000000000000,
            0.00000000000000000000,
            0.00000000000000000000,
    };
    c_float u[3] = {
            1.00000000000000000000,
            0.69999999999999995559,
            0.69999999999999995559,
    };
    c_int n = 2;
    c_int m = 3;

    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
    return data;
}

OSQPSettings *SolverDefaultSettings() {
    // Define Solver default settings
    OSQPSettings *settings =
            reinterpret_cast<OSQPSettings *>(malloc(sizeof(OSQPSettings)));
    // (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

    osqp_set_default_settings(settings);
    settings->alpha = 1.0;
    //   settings->polish = true;
    //   settings->verbose = false;
    //   settings->scaled_termination = true;
    return settings;
}

void Optimize_1() {
    OSQPData *data = FormulateProblem();

    OSQPSettings *settings = SolverDefaultSettings();
    settings->max_iter = 40;

    OSQPWorkspace *osqp_work = osqp_setup(data, settings);

    osqp_solve(osqp_work);
    // Solve Problem
    osqp_solve(osqp_work);

    // Cleanup
    osqp_cleanup(osqp_work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
}

int main(int argc, char **argv) {
    //   Load problem data
    // c_float P_x[4] =
    // { 4.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
    //   2.00000000000000000000, };
    // c_int   P_nnz  = 4;
    // c_int   P_i[4] = { 0, 1, 0, 1, };
    // c_int   P_p[3] = { 0, 2, 4, };
    // c_float q[2]   = { 1.00000000000000000000, 1.00000000000000000000, };
    // c_float A_x[4] =
    // { 1.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
    //   1.00000000000000000000, };
    // c_int   A_nnz  = 4;
    // c_int   A_i[4] = { 0, 1, 0, 2, };
    // c_int   A_p[3] = { 0, 2, 4, };
    // c_float l[3]   =
    // { 1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000,
    // }; c_float u[3] = { 1.00000000000000000000, 0.69999999999999995559,
    // 0.69999999999999995559, }; c_int n = 2; c_int m = 3;

    // // Problem settings
    // OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

    // // Structures
    // OSQPWorkspace *work; // Workspace
    // OSQPData *data;      // OSQPData

    // // Populate data
    // data    = (OSQPData *)c_malloc(sizeof(OSQPData));
    // data->n = n;
    // data->m = m;
    // data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    // data->q = q;
    // data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    // data->l = l;
    // data->u = u;

    // // Define Solver settings as default
    // osqp_set_default_settings(settings);

    // // Setup workspace
    // work = osqp_setup(data, settings);

    // // Solve Problem
    // osqp_solve(work);

    // // Clean workspace
    // osqp_cleanup(work);
    // c_free(data->A);
    // c_free(data->P);
    // c_free(data);
    // c_free(settings);

    //   std::vector<int> v(10, 0);
    //   std::cout << v[10] << std::endl;
    Optimize_1();

    return 0;
}