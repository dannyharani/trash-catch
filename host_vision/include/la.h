#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <assert.h>

#define LA_MAX_DIM 6
#define SINGULAR_EPSILON 1e-15

typedef struct {
    size_t rows;
    size_t cols;
    double data[LA_MAX_DIM * LA_MAX_DIM]; /* Max 6x6 (3D space/velocity)*/
} tMatrix;

static inline double* la_at(tMatrix *m, size_t row, size_t col) {
    assert(m != NULL && row < m->rows && col < m->cols);
    return &m->data[row * m->cols + col];
}

static inline double la_at_const(const tMatrix *m, size_t row, size_t col) {
    assert(m != NULL && row < m->rows && col < m->cols);
    return m->data[row * m->cols + col];
}

static inline double la_get(const tMatrix *m, size_t row, size_t col) {
    return la_at_const(m, row, col);
}

static inline void la_set(tMatrix *m, size_t row, size_t col, double val) {
    *la_at(m, row, col) = val;
}

tMatrix la_new(size_t rows, size_t cols);
tMatrix la_identity(size_t n);
tMatrix la_sum(tMatrix a, tMatrix b);
tMatrix la_diff(tMatrix a, tMatrix b);
tMatrix la_transpose(tMatrix a);
tMatrix la_mult(tMatrix a, tMatrix b);
tMatrix la_inverse_3x3(tMatrix a);
