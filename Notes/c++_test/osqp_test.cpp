/*
 * @Author: wangdezhao
 * @Date: 2022-04-14 01:10:20
 * @LastEditTime: 2022-04-14 14:39:20
 * @FilePath: /osqp_eigen/src/osqp_test.cpp
 * @Copyright:
 */
#include "osqp/osqp.h"

#include <iostream>
#include <vector>
#include <stdlib.h>
#include "stdio.h"


template<typename T>
T *copy_(T *vec) {
    T *data = new T[sizeof(vec)];
    memcpy(data, vec, sizeof(vec));
    return data;
}

template <typename T>
T* copy(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
}

OSQPData *FormulateProblem() {
    OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

    std::vector<c_float> P_x = { 4.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
                                 2.00000000000000000000};

    std::cout << "p"<<sizeof (P_x)<<std::endl;
    c_int   P_nnz  = 4;
//    c_int   P_i[4] = { 0, 1, 0, 1, };
    std::vector<c_int> P_i = { 0, 1, 0, 1};

//    c_int   P_p[3] = { 0, 2, 4, };
    std::vector<c_int> P_p = { 0, 2, 4};
//    c_float q[2]   = { 1.00000000000000000000, 1.00000000000000000000, };
    std::vector<c_float> q= { 1.00000000000000000000, 1.00000000000000000000};
//    c_float A_x[4] =
//            { 1.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
//              1.00000000000000000000, };
    std::vector<c_float> A_x= { 1.00000000000000000000, 1.00000000000000000000, 1.00000000000000000000,
                                1.00000000000000000000};

    c_int   A_nnz  = 4;
//    c_int   A_i[4] = { 0, 1, 0, 2, };
    std::vector<c_int> A_i = { 0, 1, 0, 2 };
//    c_int   A_p[3] = { 0, 2, 4, };
    std::vector<c_int> A_p = {0, 2, 4};
//    c_float l[3]   =
//            { 1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000, };
    std::vector<c_float> l = {1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000};
//    c_float u[3] =
//            { 1.00000000000000000000, 0.69999999999999995559, 0.69999999999999995559, };
    std::vector<c_float> u = {1.00000000000000000000, 0.69999999999999995559, 0.69999999999999995559};

    c_int n = 2;
    c_int m = 3;
//
//    std::vector<c_float> A_data;
//    std::vector<c_int> A_indices;
//    std::vector<c_int> A_indptr;
//    std::vector<c_float> lower_bounds;
//    std::vector<c_float> u;
//    std::vector<c_float> l;

    if (data) {
        data->n = n;
        data->m = m;
//        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->P = csc_matrix(data->n, data->n, P_nnz, copy(P_x), copy(P_i), copy(P_p));

//        data->q = q;
        data->q = copy(q);

//        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->A = csc_matrix(data->m, data->n, A_nnz, copy(A_x), copy(A_i), copy(A_p));

        data->l = copy(l);
        data->u = copy(u);
    }
//    if (data) {
//        data->n = n;
//        data->m = m;
//        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
//        data->q = q;
//        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
//        data->l = l;
//        data->u = u;
//    }
        return data;
}


OSQPSettings *SolverDefaultSettings() {
    // Define Solver default settings
//    OSQPSettings *settings =
//            reinterpret_cast<OSQPSettings *>(malloc(sizeof(OSQPSettings)));
//            (OSQPSettings *) c_malloc(sizeof(OSQPSettings));
    OSQPSettings *settings = (OSQPSettings *) c_malloc(sizeof(OSQPSettings));

    if (settings) osqp_set_default_settings(settings);

    settings->alpha = 1.0;
    settings->polish = true;
    settings->verbose = true;
    settings->scaled_termination = true;
    return settings;
}

void Optimize() {
    OSQPData *data = FormulateProblem();

    OSQPSettings *settings = SolverDefaultSettings();
    settings->max_iter = 4000;

    OSQPWorkspace *osqp_work = osqp_setup(data, settings);

    osqp_solve(osqp_work);

    // Clean workspace
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
    Optimize();

    return 0;
}