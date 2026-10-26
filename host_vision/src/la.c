#include "la.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>

static inline size_t idx(size_t cols, size_t r, size_t c) {
    return r * cols + c;
}

tMatrix la_new(size_t rows, size_t cols) {
    assert(rows <= LA_MAX_DIM && cols <= LA_MAX_DIM);
    tMatrix m = { .rows = rows, .cols = cols, .data = {0} };
    return m;
}

tMatrix la_identity(size_t n) {
    tMatrix res = la_new(n, n);
    for (size_t i = 0; i < n; i++) {
        res.data[idx(n, i, i)] = 1.0;
    }
    return res;
}

tMatrix la_sum(tMatrix a, tMatrix b) {
    if (a.rows != b.rows || a.cols != b.cols) return la_new(0, 0);

    tMatrix res = la_new(a.rows, a.cols);
    size_t n = a.rows * a.cols;
    for (size_t i = 0; i < n; i++) {
        res.data[i] = a.data[i] + b.data[i];
    }
    return res;
}

tMatrix la_diff(tMatrix a, tMatrix b) {
    if (a.rows != b.rows || a.cols != b.cols) return la_new(0, 0);

    tMatrix res = la_new(a.rows, a.cols);
    size_t n = a.rows * a.cols;
    for (size_t i = 0; i < n; i++) {
        res.data[i] = a.data[i] - b.data[i];
    }
    return res;
}

tMatrix la_transpose(tMatrix a) {
    tMatrix res = la_new(a.cols, a.rows);
    for (size_t r = 0; r < a.rows; r++) {
        for (size_t c = 0; c < a.cols; c++) {
            res.data[idx(res.cols, c, r)] = a.data[idx(a.cols, r, c)];
        }
    }
    return res;
}

tMatrix la_mult(tMatrix a, tMatrix b) {
    if (a.cols != b.rows) return la_new(0, 0);

    tMatrix res = la_new(a.rows, b.cols);
    for (size_t r = 0; r < a.rows; r++) {
        for (size_t k = 0; k < a.cols; k++) {
            double v = a.data[idx(a.cols, r, k)];
            for (size_t c = 0; c < b.cols; c++) {
                res.data[idx(res.cols, r, c)] += v * b.data[idx(b.cols, k, c)];
            }
        }
    }
    return res;
}

tMatrix la_inverse_3x3(tMatrix a) {
    if (a.rows != 3 || a.cols != 3) return la_new(0, 0);

    double a00 = a.data[0], a01 = a.data[1], a02 = a.data[2];
    double a10 = a.data[3], a11 = a.data[4], a12 = a.data[5];
    double a20 = a.data[6], a21 = a.data[7], a22 = a.data[8];

    double c00 =  (a11 * a22 - a12 * a21);
    double c01 = -(a10 * a22 - a12 * a20);
    double c02 =  (a10 * a21 - a11 * a20);

    double det = a00 * c00 + a01 * c01 + a02 * c02;
    if (fabs(det) < SINGULAR_EPSILON) return la_new(0, 0);

    double c10 = -(a01 * a22 - a02 * a21);
    double c11 =  (a00 * a22 - a02 * a20);
    double c12 = -(a00 * a21 - a01 * a20);

    double c20 =  (a01 * a12 - a02 * a11);
    double c21 = -(a00 * a12 - a02 * a10);
    double c22 =  (a00 * a11 - a01 * a10);

    double inv_det = 1.0 / det;
    tMatrix res = la_new(3, 3);

    res.data[0] = c00 * inv_det;
    res.data[1] = c10 * inv_det;
    res.data[2] = c20 * inv_det;

    res.data[3] = c01 * inv_det;
    res.data[4] = c11 * inv_det;
    res.data[5] = c21 * inv_det;

    res.data[6] = c02 * inv_det;
    res.data[7] = c12 * inv_det;
    res.data[8] = c22 * inv_det;

    return res;
}
