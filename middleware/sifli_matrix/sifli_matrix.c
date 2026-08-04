/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <string.h>
#include "sifli_matrix.h"

//#define SIFLI_MATRIX_DEBUG
#include "rtconfig.h"

#if defined(SIFLI_MATRIX_DEBUG) && !defined(CFG_BOOTLOADER)

extern void rt_kprintf(const char *fmt, ...);

#define SIFLI_MATRIX_PRINT(...)   do{\
                                        rt_kprintf(__VA_ARGS__);\
                                        rt_kprintf("\n");\
                                    }while(0)
#else
#define SIFLI_MATRIX_PRINT(...)
#endif /* SIFLI_MATRIX_DEBUG */



#ifndef M_PI
    #define M_PI 3.1415926f
#endif /* M_PI */

#define SIFLI_MATRIX_EPSILON  ((float)(1e-6))

#define SIFLI_MATRIX_IS_EQUAL_TO_ZERO(x)   (((float)(x) < SIFLI_MATRIX_EPSILON) && ((float)(x) > -SIFLI_MATRIX_EPSILON))

#ifdef SIFLI_MATRIX_DEBUG
static void print_vertex(const char *s, const sifli_vertex_3d_t *vertex)
{
    SIFLI_MATRIX_PRINT("%s:%10.3f,%10.3f,%10.3f", s, vertex->x, vertex->y, vertex->z);
}

static void print_vertex_2d(const char *s, const sifli_vertex_2d_t *vertex)
{
    SIFLI_MATRIX_PRINT("%s:%10.3f,%10.3f", s, vertex->x, vertex->y);
}

static void print_matrix_4x4(const char *s, const sifli_matrix_4x4_t *mat)
{
    SIFLI_MATRIX_PRINT("%s", s);
    SIFLI_MATRIX_PRINT("%10.3f  %10.3f  %10.3f  %10.3f", mat->m[0][0], mat->m[0][1], mat->m[0][2], mat->m[0][3]);
    SIFLI_MATRIX_PRINT("%10.3f  %10.3f  %10.3f  %10.3f", mat->m[1][0], mat->m[1][1], mat->m[1][2], mat->m[0][3]);
    SIFLI_MATRIX_PRINT("%10.3f  %10.3f  %10.3f  %10.3f", mat->m[2][0], mat->m[2][1], mat->m[2][2], mat->m[0][3]);
    SIFLI_MATRIX_PRINT("%10.3f  %10.3f  %10.3f  %10.3f", mat->m[3][0], mat->m[3][1], mat->m[3][2], mat->m[3][3]);
}

static void print_matrix(const char *s, const sifli_matrix_3x3_t *mat)
{
    SIFLI_MATRIX_PRINT("%s", s);
    SIFLI_MATRIX_PRINT("%10.3f,%10.3f,%10.3f", mat->m[0][0], mat->m[0][1], mat->m[0][2]);
    SIFLI_MATRIX_PRINT("%10.3f,%10.3f,%10.3f", mat->m[1][0], mat->m[1][1], mat->m[1][2]);
    SIFLI_MATRIX_PRINT("%10.3f,%10.3f,%10.3f", mat->m[2][0], mat->m[2][1], mat->m[2][2]);
}
#else
#define print_vertex(s, vertex)
#define print_vertex_2d(s, vertex)
#define print_matrix_4x4(s, mat)
#define print_matrix(s, mat)
#endif /* SIFLI_MATRIX_DEBUG */

void sifli_identity_3x3(sifli_matrix_3x3_t *matrix)
{
    if (!matrix)
    {
        return;
    }

    memset(matrix, 0, sizeof(*matrix));
    matrix->m[0][0] = 1.0f;
    matrix->m[1][1] = 1.0f;
    matrix->m[2][2] = 1.0f;
}
int sifli_mat_vertex_multiply(sifli_matrix_3x3_t *matrix, float *x, float *y)
{
    float x1;
    float y1;
    float w;

    x1 = matrix->m[0][0] * (*x) + matrix->m[0][1] * (*y) + matrix->m[0][2];
    y1 = matrix->m[1][0] * (*x) + matrix->m[1][1] * (*y) + matrix->m[1][2];
    w = matrix->m[2][0] * (*x) + matrix->m[2][1] * (*y) + matrix->m[2][2];

    if (w == 0.0f)
    {
        return 0;
    }

    if (w != 1.0f)
    {
        *x = x1 / w;
        *y = y1 / w;
    }
    else
    {
        *x = x1;
        *y = y1;
    }

    return 1;
}

sifli_error_t sifli_mat_vertex_multiply_4x4(sifli_matrix_4x4_t *matrix, sifli_vertex_3d_t *input, sifli_vertex_3d_t *output)
{
    sifli_vertex_3d_t temp;
    float w;

    if (!matrix || !input || !output)
    {
        return SIFLI_MATRIX_INVALID_ARGUMENT;
    }

    temp.x = matrix->m[0][0] * input->x + matrix->m[0][1] * input->y + matrix->m[0][2] * input->z + matrix->m[0][3];
    temp.y = matrix->m[1][0] * input->x + matrix->m[1][1] * input->y + matrix->m[1][2] * input->z + matrix->m[1][3];
    temp.z = matrix->m[2][0] * input->x + matrix->m[2][1] * input->y + matrix->m[2][2] * input->z + matrix->m[2][3];
    w = matrix->m[3][0] * input->x + matrix->m[3][1] * input->y + matrix->m[3][2] * input->z + matrix->m[3][3];

    if (w != 1)
    {
        temp.x /= w;
        temp.y /= w;
        temp.z /= w;
    }

    memcpy((void *)output, (void *)&temp, sizeof(*output));

    return SIFLI_MATRIX_SUCCESS;
}

void sifli_mat_vertex_multiply2_4x4(sifli_matrix_4x4_t *matrix, float *x, float *y, float *z)
{
    sifli_vertex_3d_t temp;
    float w;

    temp.x = matrix->m[0][0] * (*x) + matrix->m[0][1] * (*y) + matrix->m[0][2] * (*z) + matrix->m[0][3];
    temp.y = matrix->m[1][0] * (*x) + matrix->m[1][1] * (*y) + matrix->m[1][2] * (*z) + matrix->m[1][3];
    temp.z = matrix->m[2][0] * (*x) + matrix->m[2][1] * (*y) + matrix->m[2][2] * (*z) + matrix->m[2][3];
    w = matrix->m[3][0] * (*x) + matrix->m[3][1] * (*y) + matrix->m[3][2] * (*z) + matrix->m[3][3];

    if (w != 1)
    {
        temp.x /= w;
        temp.y /= w;
        temp.z /= w;
    }

    *x = temp.x;
    *y = temp.y;
    *z = temp.z;
}



void sifli_mat_multiply_3x3(sifli_matrix_3x3_t *matrix, sifli_matrix_3x3_t *mult, sifli_matrix_3x3_t *multed)
{
    sifli_matrix_3x3_t temp;
    int row, column;

    /* Process all rows. */
    for (row = 0; row < 3; row++)
    {
        /* Process all columns. */
        for (column = 0; column < 3; column++)
        {
            /* Compute matrix entry. */
            temp.m[row][column] = (matrix->m[row][0] * mult->m[0][column])
                                  + (matrix->m[row][1] * mult->m[1][column])
                                  + (matrix->m[row][2] * mult->m[2][column]);
        }
    }

    /* Copy temporary matrix into result. */
    memcpy(multed, &temp, sizeof(temp));
}

void sifli_mat_translate_3x3(float translate_x, float translate_y, sifli_matrix_3x3_t *matrix)
{
    /* Set translation matrix. */
    sifli_matrix_3x3_t t = { { {1.0f, 0.0f, translate_x},
            {0.0f, 1.0f, translate_y},
            {0.0f, 0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply(matrix, &t);
}

void sifli_mat_scale_3x3(float scale_x, float scale_y, sifli_matrix_3x3_t *matrix)
{
    /* Set scale matrix. */
    sifli_matrix_3x3_t s = { {
            {scale_x, 0.0f, 0.0f},
            {0.0f, scale_y, 0.0f},
            {0.0f, 0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply(matrix, &s);
}

void sifli_mat_rotate_3x3(float degrees, sifli_matrix_3x3_t *matrix)
{
    float radians;
    float cos_angle, sin_angle;

    /* Convert degrees into radians. */
    radians = degrees  / 180.0f * M_PI;

    /* Calculate cosine and sine of angle. */
    cos_angle = cosf(radians);
    sin_angle = sinf(radians);

    /* Set rotation matrix. */
    sifli_matrix_3x3_t r = { {
            {cos_angle, sin_angle, 0.0f},
            {-sin_angle,  cos_angle, 0.0f},
            {0.0f,       0.0f,      1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply(matrix, &r);
}

void sifli_mat_skew_3x3(float degrees_x, float degrees_y, sifli_matrix_3x3_t *matrix)
{
    float radians_x, radians_y;
    float tan_x, tan_y;

    /* Convert degrees into radians. */
    radians_x = degrees_x  / 180.0f * M_PI;
    radians_y = degrees_y  / 180.0f * M_PI;

    /* Calculate tangent of angles. */
    tan_x = tanf(radians_x);
    tan_y = tanf(radians_y);

    /* Set skew matrix. */
    sifli_matrix_3x3_t sk = { {
            {1.0f, tan_x, 0.0f},
            {tan_y, 1.0f, 0.0f},
            {0.0f,  0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply(matrix, &sk);
}

int sifli_mat_inverse_3x3(const sifli_matrix_3x3_t *input, sifli_matrix_3x3_t *output)
{
    float det;

    /* Calculate the determinant of the matrix. */
    det = input->m[0][0] * (input->m[1][1] * input->m[2][2] - input->m[1][2] * input->m[2][1])
          - input->m[0][1] * (input->m[1][0] * input->m[2][2] - input->m[1][2] * input->m[2][0])
          + input->m[0][2] * (input->m[1][0] * input->m[2][1] - input->m[1][1] * input->m[2][0]);

    if (SIFLI_MATRIX_IS_EQUAL_TO_ZERO(det))
    {
        /* Matrix is not invertible, return identity matrix. */
        output->m[0][0] = 1.0f;
        output->m[0][1] = 0.0f;
        output->m[0][2] = 0.0f;
        output->m[1][0] = 0.0f;
        output->m[1][1] = 1.0f;
        output->m[1][2] = 0.0f;
        output->m[2][0] = 0.0f;
        output->m[2][1] = 0.0f;
        output->m[2][2] = 1.0f;

        SIFLI_MATRIX_PRINT("Matrix not invertible, determinant is zero.\n");
        return 1;
    }
    else
    {
        float inv_det = 1.0f / det;

        /* Calculate inverse using Cramer's rule. */
        output->m[0][0] = (input->m[1][1] * input->m[2][2] - input->m[1][2] * input->m[2][1]) * inv_det;
        output->m[0][1] = -(input->m[0][1] * input->m[2][2] - input->m[0][2] * input->m[2][1]) * inv_det;
        output->m[0][2] = (input->m[0][1] * input->m[1][2] - input->m[0][2] * input->m[1][1]) * inv_det;
        output->m[1][0] = -(input->m[1][0] * input->m[2][2] - input->m[1][2] * input->m[2][0]) * inv_det;
        output->m[1][1] = (input->m[0][0] * input->m[2][2] - input->m[0][2] * input->m[2][0]) * inv_det;
        output->m[1][2] = -(input->m[0][0]  * input->m[1][2] - input->m[0][2] * input->m[1][0]) * inv_det;
        output->m[2][0] = (input->m[1][0] * input->m[2][1] - input->m[1][1] * input->m[2][0]) * inv_det;
        output->m[2][1] = -(input->m[0][0] * input->m[2][1] - input->m[0][1] * input->m[2][0]) * inv_det;
        output->m[2][2] = (input->m[0][0] * input->m[1][1] - input->m[0][1] * input->m[1][0]) * inv_det;

        print_matrix("inverse matrix(input)", input);
        print_matrix("inverse matrix(output)", output);
        return 0;
    }
}


void sifli_multiply_4x4(sifli_matrix_4x4_t *matrix, sifli_matrix_4x4_t *mult)
{
    sifli_matrix_4x4_t temp;
    int row, column;

    /* Process all rows. */
    for (row = 0; row < 4; row++)
    {
        /* Process all columns. */
        for (column = 0; column < 4; column++)
        {
            /* Compute matrix entry. */
            temp.m[row][column] = (matrix->m[row][0] * mult->m[0][column])
                                  + (matrix->m[row][1] * mult->m[1][column])
                                  + (matrix->m[row][2] * mult->m[2][column])
                                  + (matrix->m[row][3] * mult->m[3][column]);
        }
    }

    /* Copy temporary matrix into result. */
    memcpy(matrix, &temp, sizeof(temp));
}

void sifli_vec_cross_multiply(sifli_vec_3d_t *product, sifli_vec_3d_t *l, sifli_vec_3d_t *r)
{
    sifli_vec_3d_t temp;

    temp.x = l->y * r->z - l->z * r->y;
    temp.y = l->z * r->x - l->x * r->z;
    temp.z = l->x * r->y - l->y * r->x;

    product->x = temp.x;
    product->y = temp.y;
    product->z = temp.z;
}

void sifli_mirror_x(sifli_matrix_3x3_t *matrix)
{
    sifli_mirror(0.0f, 1.0f, matrix);
}

void sifli_mirror_y(sifli_matrix_3x3_t *matrix)
{
    sifli_mirror(1.0f, 0.0f, matrix);
}

void sifli_mirror(float nx, float ny, sifli_matrix_3x3_t *matrix)
{
    /* Set mirror matrix. */
    sifli_matrix_3x3_t t = { {
            {1.0f - 2.0f * (nx * nx), -2.0f * nx * ny,         0.0f},
            {-2.0f * nx * ny,         1.0f - 2.0f * (ny * ny), 0.0f},
            {0.0f,                    0.0f,                    1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply(matrix, &t);
}

void sifli_identity_4x4(sifli_matrix_4x4_t *matrix)
{
    /* Set identify matrix. */
    matrix->m[0][0] = 1.0f;
    matrix->m[0][1] = 0.0f;
    matrix->m[0][2] = 0.0f;
    matrix->m[0][3] = 0.0f;
    matrix->m[1][0] = 0.0f;
    matrix->m[1][1] = 1.0f;
    matrix->m[1][2] = 0.0f;
    matrix->m[1][3] = 0.0f;
    matrix->m[2][0] = 0.0f;
    matrix->m[2][1] = 0.0f;
    matrix->m[2][2] = 1.0f;
    matrix->m[2][3] = 0.0f;
    matrix->m[3][0] = 0.0f;
    matrix->m[3][1] = 0.0f;
    matrix->m[3][2] = 0.0f;
    matrix->m[3][3] = 1.0f;
}

void sifli_mirror_4x4(float nx, float ny, float nz, sifli_matrix_4x4_t *matrix)
{
    /* Set mirror matrix. */
    sifli_matrix_4x4_t t = { {
            {1.0f - 2.0f * (nx * nx), -2.0f * nx * ny,         -2.0f * nx * nz,           0.0f},
            {-2.0f * nx * ny,          1.0f - 2.0f * (ny * ny), -2.0f * ny * nz,           0.0f},
            {-2.0f * nx * nz,         -2.0f * ny * nz,          1.0f - 2.0f * (nz * nz),  0.0f},
            {0.0f,                     0.0f,                    0.0f,                     1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &t);
}

void sifli_translate_4x4(float x, float y, float z, sifli_matrix_4x4_t *matrix)
{
    /* Set translation matrix. */
    sifli_matrix_4x4_t t = { { {1.0f, 0.0f, 0.0f, x},
            {0.0f, 1.0f, 0.0f, y},
            {0.0f, 0.0f, 1.0f, z},
            {0.0f, 0.0f, 0.0f, 1}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &t);
}

void sifli_scale_4x4(float scale_x, float scale_y, float scale_z, sifli_matrix_4x4_t *matrix)
{
    /* Set scale matrix. */
    sifli_matrix_4x4_t s = { { {scale_x, 0.0f, 0.0f, 0.0f},
            {0.0f, scale_y, 0.0f, 0.0f},
            {0.0f, 0.0f, scale_z, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &s);
}

void sifli_rotate_x(float degrees, sifli_matrix_4x4_t *matrix)
{
    /* Convert degrees into radians. */
    float angle = degrees / 180.0f * M_PI;

    /* Compuet cosine and sine values. */
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);

    /* Set rotation matrix. */
    sifli_matrix_4x4_t r = { {
            {1.0f, 0.0f, 0.0f, 0.0f},
            {0.0f, cos_angle, -sin_angle, 0.0f},
            {0.0f, sin_angle, cos_angle, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &r);
}

void sifli_rotate_y(float degrees, sifli_matrix_4x4_t *matrix)
{
    /* Convert degrees into radians. */
    float angle = degrees / 180.0f * M_PI;

    /* Compuet cosine and sine values. */
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);

    /* Set rotation matrix. */
    sifli_matrix_4x4_t r = { {
            {cos_angle, 0.0f, sin_angle, 0.0f},
            {0.0f, 1.0f, 0.0f, 0.0f},
            {-sin_angle, 0.0f, cos_angle, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f}
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &r);
}

void sifli_rotate_z(float degrees, sifli_matrix_4x4_t *matrix)
{
    /* Convert degrees into radians. */
    float angle = degrees / 180.0f * M_PI;

    /* Compuet cosine and sine values. */
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);

    /* Set rotation matrix. */
    sifli_matrix_4x4_t r = { {
            {cos_angle, -sin_angle, 0.0f, 0.0f},
            {sin_angle, cos_angle, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 1.0f},
        }
    };

    /* Multiply with current matrix. */
    sifli_multiply_4x4(matrix, &r);
}

void sifli_perspective_4x4(float fov_degrees, float aspect,
                           float near_val, float far_val, sifli_matrix_4x4_t *matrix)
{
    float scaling_factor = sifli_get_perspective_scaling_factor(fov_degrees);

    /* Set perspective matrix. */
    matrix->m[0][0] = scaling_factor / aspect;
    matrix->m[0][1] = 0.0f;
    matrix->m[0][2] = 0.0f;
    matrix->m[0][3] = 0.0f;
    matrix->m[1][0] = 0.0f;
    matrix->m[1][1] = scaling_factor;
    matrix->m[1][2] = 0.0f;
    matrix->m[1][3] = 0.0f;
    matrix->m[2][0] = 0.0f;
    matrix->m[2][1] = 0.0f;
    matrix->m[2][2] = (-near_val - far_val) / (far_val - near_val);
    matrix->m[2][3] = (2.0f * near_val * far_val) / (near_val - far_val);
    matrix->m[3][0] = 0.0f;
    matrix->m[3][1] = 0.0f;
    matrix->m[3][2] = -1.0f;
    matrix->m[3][3] = 0.0f;
}

sifli_error_t sifli_perspective_3x3(float fov_degrees, int width, int height, sifli_quad_3d_t quad, sifli_matrix_3x3_t *matrix)
{
    float scaling_factor = sifli_get_perspective_scaling_factor(fov_degrees);
    sifli_error_t err;

    err = sifli_perspective2_3x3(scaling_factor, width, height, &quad[0], &quad[1], &quad[2], &quad[3], matrix);

    return err;
}

sifli_error_t sifli_perspective2_3x3(float scaling_factor, int width, int height, sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1,
                                     sifli_vertex_3d_t *v2, sifli_vertex_3d_t *v3, sifli_matrix_3x3_t *matrix)
{
    sifli_vec_3d_t normal;
    float w0, w1, w2;

    sifli_get_normal(v0, v1, v2, &normal);

    /* -z = w0*x + w1*y + w2 */
    if (SIFLI_MATRIX_IS_EQUAL_TO_ZERO(normal.z))
    {
        return SIFLI_MATRIX_INVALID_ARGUMENT;
    }
    w0 = normal.x / normal.z;
    w1 = normal.y / normal.z;
    w2 = -(normal.x * v0->x + normal.y * v0->y + normal.z * v0->z) / normal.z;

    /* Set perspective matrix. */
    matrix->m[0][0] = ((float)height * scaling_factor + w0 * (float)width) * 0.5f;
    matrix->m[0][1] = w1 * (float)width * 0.5f;
    matrix->m[0][2] = w2 * (float)width * 0.5f;
    matrix->m[1][0] = w0 * (float)height * 0.5f;
    matrix->m[1][1] = ((float) - height * scaling_factor + w1 * (float)height) * 0.5f;
    matrix->m[1][2] = w2 * (float)height * 0.5f;
    matrix->m[2][0] = w0;
    matrix->m[2][1] = w1;
    matrix->m[2][2] = w2;

    return SIFLI_MATRIX_SUCCESS;
}

sifli_error_t sifli_get_texture_map_matrix(float fov, float img_w, float img_h, float canvas_w, float canvas_h,
        sifli_quad_3d_t quad, sifli_matrix_3x3_t *matrix)

{
    sifli_error_t err;
    if (0.0f == fov) return SIFLI_MATRIX_INVALID_ARGUMENT;

    float scale = sifli_get_perspective_scaling_factor(fov);

    err = sifli_get_texture_map_matrix2(scale, img_w, img_h, canvas_w, canvas_h,
                                        &quad[0], &quad[1], &quad[2], &quad[3], matrix);

    return err;
}



sifli_error_t sifli_get_texture_map_matrix2(float scale, float img_w, float img_h, float canvas_w, float canvas_h,
        sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1, sifli_vertex_3d_t *v2,
        sifli_vertex_3d_t *v3, sifli_matrix_3x3_t *matrix)
{
    sifli_vertex_3d_t normal;
    sifli_matrix_3x3_t affine_mat;
    /* line equation: x = m*y+n */
    float m, n;
    sifli_error_t err = SIFLI_MATRIX_SUCCESS;

    sifli_get_normal(v0, v1, v2, &normal);
    print_vertex("normal", &normal);
    SIFLI_MATRIX_PRINT("normal.z:%f\n", normal.z);

    if (!SIFLI_MATRIX_IS_EQUAL_TO_ZERO(normal.z))
    {
        // sifli_calc_affine_matrix(img_w, img_h, &quad[2], &quad[3], &quad[0], &quad[1], &affine_mat);
        sifli_get_affine_matrix2(img_w, img_h, &v0->x, &v0->y, &v1->x, &v1->y, &v2->x, &v2->y,
                                 &v3->x, &v3->y, &affine_mat);
        // sifli_calc_affine_matrix(img_w, img_h, &quad[1], &quad[0], &quad[3], &quad[2], &affine_mat);  // coverflow
        // sifli_calc_affine_matrix(img_w, img_h, &quad[0], &quad[3], &quad[2], &quad[1], &affine_mat); //90 degrees

        print_matrix("affine matrix", &affine_mat);

        err = sifli_perspective2_3x3(scale, canvas_w, canvas_h, v0, v1, v2, v3, matrix);
        if (SIFLI_MATRIX_SUCCESS != err)
        {
            goto __EXIT;
        }
    }
    else if (SIFLI_MATRIX_IS_EQUAL_TO_ZERO(normal.y))
    {
        // sifli_calc_affine_matrix(img_w, img_h, &quad_n[1], &quad_n[0], &quad_n[3], &quad_n[2], &affine_mat);
        sifli_get_affine_matrix2(img_w, img_h, &v0->y, &v0->z, &v1->y, &v1->z, &v2->y, &v2->z, &v3->y, &v3->z, &affine_mat);
        print_matrix("affine matrix2-0", &affine_mat);

        matrix->m[0][0] = 0;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = v0->x * scale;
        matrix->m[1][0] = -scale;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = 0;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }
    else if (SIFLI_MATRIX_IS_EQUAL_TO_ZERO(normal.x))
    {
        sifli_get_affine_matrix2(img_w, img_h, &v0->x, &v0->z, &v1->x, &v1->z, &v2->x, &v2->z, &v3->x, &v3->z, &affine_mat);
        print_matrix("affine matrix2-1", &affine_mat);

        matrix->m[0][0] = scale;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = 0;
        matrix->m[1][0] = 0;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = -v0->y * scale;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }
    else
    {
        sifli_get_affine_matrix2(img_w, img_h, &v0->y, &v0->z, &v1->y, &v1->z, &v2->y, &v2->z, &v3->y, &v3->z, &affine_mat);
        print_matrix("affine matrix2-2", &affine_mat);

        m = -normal.y / normal.x;
        n = (normal.x * v0->x + normal.y * v0->y) / normal.x;
        matrix->m[0][0] = m * scale;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = n * scale;
        matrix->m[1][0] = -scale;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = 0;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }

    print_matrix("proj matrix", matrix);
    sifli_multiply(matrix, &affine_mat);
    print_matrix("final matrix", matrix);

__EXIT:
    return err;
}

/** \brief Convenience Function to calculate window coordinates from object coordinates
 *
 * \param mvp Model, View and Projection Matrix
 * \param x_orig Window top left X coordinate
 * \param y_orig Window top left Y coordinate
 * \param width Window width
 * \param height Window height
 * \param nearVal Distance from the viewer to the near clipping plane (always positive)
 * \param farVal Distance from the viewer to the far clipping plane (always positive)
 * \param x X object coordinate
 * \param y Y object coordinate
 * \param z Z object coordinate
 * \param w W object coordinate
 * \return 1 if vertex is outside frustum (should be clipped)
 *
 */
int sifli_4x4_obj_to_ndc_coords(sifli_matrix_4x4_t *matrix,
                                float x_orig,  float y_orig,
                                int width, int height,
                                float nearVal, float farVal,
                                float *x,
                                float *y,
                                float *z,
                                float *w)
{
    int clipped;
    float x_ndc, y_ndc, z_ndc, w_ndc;

    /* Compute matrix entry. */
    x_ndc = (matrix->m[0][0] * (*x))
            + (matrix->m[0][1] * (*y))
            + (matrix->m[0][2] * (*z))
            + (matrix->m[0][3] * (*w));

    y_ndc = (matrix->m[1][0] * (*x))
            + (matrix->m[1][1] * (*y))
            + (matrix->m[1][2] * (*z))
            + (matrix->m[1][3] * (*w));

    z_ndc = (matrix->m[2][0] * (*x))
            + (matrix->m[2][1] * (*y))
            + (matrix->m[2][2] * (*z))
            + (matrix->m[2][3] * (*w));


    w_ndc = (matrix->m[3][0] * (*x))
            + (matrix->m[3][1] * (*y))
            + (matrix->m[3][2] * (*z))
            + (matrix->m[3][3] * (*w));

    if (w_ndc != 1)
    {
        *x = x_ndc / w_ndc;
        *y = y_ndc / w_ndc;
        *z = y_ndc / w_ndc;
        *w = 1;
    }
    else
    {
        *x = x_ndc;
        *y = y_ndc;
        *z = y_ndc;
        *w = w_ndc;
    }

    if ((*x < -1) || (*x > 1) || (*y < -1) || (*y > 1) || (*z < -1) || (*z > 1))  //TODO: aspect ratio should be taken into account
    {
        clipped = 1;
    }
    else
    {
        clipped = 0;
    }

    return clipped;
}


/** \brief Convenience Function to calculate window coordinates from object coordinates
 *
 * \param x_orig Window top left X coordinate
 * \param y_orig Window top left Y coordinate
 * \param width Window width
 * \param height Window height
 * \param x X object coordinate
 * \param y Y object coordinate
 * \return 1 if vertex is outside frustum (should be clipped)
 *
 */
void sifli_4x4_ndc_to_win_coords(float x_orig,  float y_orig,
                                 int width, int height,
                                 float *x,
                                 float *y)
{
    *x = (*x + 1) * 0.5f * width + x_orig;
    *y = ((1 - (*y + 1) * 0.5f) * height) + y_orig;
}


void sifli_get_normal(sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1, sifli_vertex_3d_t *v2, sifli_vertex_3d_t *normal)
{
    sifli_vec_3d_t vec1, vec2;

    vec1.x = v1->x - v0->x;
    vec1.y = v1->y - v0->y;
    vec1.z = v1->z - v0->z;
    vec2.x = v2->x - v0->x;
    vec2.y = v2->y - v0->y;
    vec2.z = v2->z - v0->z;

    sifli_vec_cross_multiply(normal, &vec1, &vec2);
}

void sifli_get_affine_matrix(float w, float h, sifli_vertex_2d_t *v0, sifli_vertex_2d_t *v1, sifli_vertex_2d_t *v2,
                             sifli_vertex_2d_t *v3, sifli_matrix_3x3_t *matrix)
{
    sifli_get_affine_matrix2(w, h, &v0->x, &v0->y, &v1->x, &v1->y, &v2->x, &v2->y, &v3->x, &v3->y, matrix);
}


void sifli_get_affine_matrix2(float w, float h, float *v0x, float *v0y,
                              float *v1x, float *v1y,
                              float *v2x, float *v2y,
                              float *v3x, float *v3y,
                              sifli_matrix_3x3_t *matrix)
{
    float sx, sy, shx, shy, tx, ty;

    /*
      To map a rectangle image (0,0),(h,0),(w,h),(0,w) to a parallelogram (v0x,v0y),(v1x,v1y),(v2x,v2y),(v3x,v3y) counterclock wise
      We get the following equations:
       x0 = tx;
       y0 = ty;
       x1 = h*shx + tx;
       y1 = h*sy + ty;
       x2 = w*sx + tx;
       y2 = w*shx + ty;
    */
    tx = *v0x;
    ty = *v0y;
    shx = (*v1x - *v0x) / h;
    sy = (*v1y - *v0y) / h;
    sx = (*v3x - *v0x) / w;
    shy = (*v3y - *v0y) / w;

    /* Set the Blit transformation matrix. */
    matrix->m[0][0] = sx;
    matrix->m[0][1] = shx;
    matrix->m[0][2] = tx;
    matrix->m[1][0] = shy;
    matrix->m[1][1] = sy;
    matrix->m[1][2] = ty;
    matrix->m[2][0] = 0.0;
    matrix->m[2][1] = 0.0;
    matrix->m[2][2] = 1.0;
}


float sifli_get_perspective_scaling_factor(float fov_degrees)
{
    /* Convert degrees into radians. */
    float angle = fov_degrees / 2.0f / 180.0f * M_PI;
    float tan_angle = tanf(angle);

    return (1.0f / tan_angle);

}

