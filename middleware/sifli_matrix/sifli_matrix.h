/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _sifli_matrix_h_
#define _sifli_matrix_h_

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************************
 * Convention:
 *   1. Use right-hand coordinates, i.e. rotation also use right-hand rule
 *   2. Counter-clockwise polygon
 *
 ***********************************************************************************/


typedef struct sifli_matrix_3x3
{
    float m[3][3];    /*! The 3x3 matrix itself, in [row][column] order. */
} sifli_matrix_3x3_t;

/** 4x4 matrix */
typedef struct sifli_matrix_4x4
{
    float m[4][4];    /*! The 4x4 matrix itself, in [row][column] order. */
} sifli_matrix_4x4_t;

/** vertex in 3d space*/
typedef struct sifli_vertex
{
    float x;
    float y;
    float z;
} sifli_vertex_3d_t;

/** vertex in 2d space */
typedef struct sifli_vertex_2d
{
    float x;
    float y;
} sifli_vertex_2d_t;



/** vector in 2d space */
typedef sifli_vertex_2d_t sifli_vec_2d_t;

/** vector in 3d space */
typedef sifli_vertex_3d_t sifli_vec_3d_t;

/** 3d quadrilateral */
typedef sifli_vertex_3d_t  sifli_quad_3d_t[4];

/** 2d quadrilateral */
typedef sifli_vertex_2d_t  sifli_quad_2d_t[4];


typedef enum
{
    SIFLI_MATRIX_SUCCESS = 0,
    SIFLI_MATRIX_INVALID_ARGUMENT,
    SIFLI_MATRIX_SINGULAR,
    SIFLI_MATRIX_ERROR
} sifli_error_t;

/*
    2D Matrix operation functions
*/
void sifli_identity_3x3(sifli_matrix_3x3_t *matrix);
int sifli_mat_vertex_multiply(sifli_matrix_3x3_t *matrix, float *x, float *y);
//multed = matrix * mult
void sifli_mat_multiply_3x3(sifli_matrix_3x3_t *matrix, sifli_matrix_3x3_t *mult, sifli_matrix_3x3_t *multed);
#define sifli_multiply(matrix, mult) sifli_mat_multiply_3x3(matrix, mult, matrix)
void sifli_mirror_x(sifli_matrix_3x3_t *matrix);
void sifli_mirror_y(sifli_matrix_3x3_t *matrix);
/** (nx,ny) is normal of mirror axis  */
void sifli_mirror(float nx, float ny, sifli_matrix_3x3_t *matrix);
void sifli_get_affine_matrix(float w, float h, sifli_vertex_2d_t *v0, sifli_vertex_2d_t *v1, sifli_vertex_2d_t *v2, sifli_vertex_2d_t *v3, sifli_matrix_3x3_t *matrix);
//return 0 is OK
int sifli_mat_inverse_3x3(const sifli_matrix_3x3_t *input, sifli_matrix_3x3_t *output);
void sifli_mat_scale_3x3(float scale_x, float scale_y, sifli_matrix_3x3_t *matrix);
void sifli_mat_translate_3x3(float translate_x, float translate_y, sifli_matrix_3x3_t *matrix);
void sifli_mat_rotate_3x3(float degrees, sifli_matrix_3x3_t *matrix);
void sifli_mat_skew_3x3(float degrees_x, float degrees_y, sifli_matrix_3x3_t *matrix);
/*
    3D Matrix operation functions
*/
sifli_error_t sifli_mat_vertex_multiply_4x4(sifli_matrix_4x4_t *matrix, sifli_vertex_3d_t *input, sifli_vertex_3d_t *output);
void sifli_mat_vertex_multiply2_4x4(sifli_matrix_4x4_t *matrix, float *x, float *y, float *z);
void sifli_multiply_4x4(sifli_matrix_4x4_t *matrix, sifli_matrix_4x4_t *mult);
void sifli_vec_cross_multiply(sifli_vec_3d_t *product, sifli_vec_3d_t *l, sifli_vec_3d_t *r);
void sifli_identity_4x4(sifli_matrix_4x4_t *matrix);
/** (nx,ny,nz) is normal of mirror plane */
void sifli_mirror_4x4(float nx, float ny, float nz, sifli_matrix_4x4_t *matrix);
void sifli_translate_4x4(float x, float y, float z, sifli_matrix_4x4_t *matrix);
void sifli_scale_4x4(float scale_x, float scale_y, float scale_z, sifli_matrix_4x4_t *matrix);
void sifli_rotate_x(float degrees, sifli_matrix_4x4_t *matrix);
void sifli_rotate_y(float degrees, sifli_matrix_4x4_t *matrix);
void sifli_rotate_z(float degrees, sifli_matrix_4x4_t *matrix);
/* aspect: ResX/ResY */
void sifli_perspective_4x4(float fovy_degrees, float aspect,
                           float near_val, float far_val, sifli_matrix_4x4_t *matrix);
/**  n=(v1-v0)x(v2-v0) */
void sifli_get_normal(sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1, sifli_vertex_3d_t *v2, sifli_vertex_3d_t *normal);


/*
    3D to 2D projection functions
*/
/** construct 3x3 perspective matrix by the quadrilateral, the quadrilateral must be on the plane and not be parallel to axis-z */
sifli_error_t sifli_perspective_3x3(float fovy_degrees, int width, int height, sifli_quad_3d_t quad, sifli_matrix_3x3_t *matrix);

/** construct 3x3 perspective matrix by the quadrilateral, the quadrilateral must be on the plane and not be parallel to axis-z */
sifli_error_t sifli_perspective2_3x3(float scaling_factor, int width, int height, sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1,
                                     sifli_vertex_3d_t *v2, sifli_vertex_3d_t *v3, sifli_matrix_3x3_t *matrix);

/** get tranform matrix for texture mapping, quad defines a quadrilateral on the plane in the 3D space.
 *
 * image point (0,0) map to quad[0], point (0,h) map to quad[1],
 * point (w,h) map to quad[2], point (w,0) map to quad[3]
 * */
sifli_error_t sifli_get_texture_map_matrix(float fov, float img_w, float img_h, float canvas_w, float canvas_h,
        sifli_quad_3d_t quad, sifli_matrix_3x3_t *matrix);


sifli_error_t sifli_get_texture_map_matrix2(float scale, float img_w, float img_h, float canvas_w, float canvas_h,
        sifli_vertex_3d_t *v0, sifli_vertex_3d_t *v1, sifli_vertex_3d_t *v2,
        sifli_vertex_3d_t *v3, sifli_matrix_3x3_t *matrix);


int sifli_4x4_obj_to_ndc_coords(sifli_matrix_4x4_t *matrix,
                                float x_orig,  float y_orig,
                                int width, int height,
                                float near_val, float far_val,
                                float *x,
                                float *y,
                                float *z,
                                float *w);

void sifli_4x4_ndc_to_win_coords(float x_orig,  float y_orig,
                                 int width, int height,
                                 float *x,
                                 float *y);




void sifli_get_affine_matrix2(float w, float h, float *v0x, float *v0y,
                              float *v1x, float *v1y,
                              float *v2x, float *v2y,
                              float *v3x, float *v3y,
                              sifli_matrix_3x3_t *matrix);

float sifli_get_perspective_scaling_factor(float fov_degrees);

#ifdef __cplusplus
}
#endif
#endif /* _sifli_matrix_h_ */
