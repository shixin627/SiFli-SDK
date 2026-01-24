/* ----------------------------------------------------------------------
* Copyright (C) 2010-2018 Arm Limited. All rights reserved.
*
*
* Project:       CMSIS NN Library
* Title:         arm_nnexamples_nn_test.cpp
*
* Description:   Example code for NN kernel testing.
*
* Target Processor: Cortex-M cores
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */
#include "rtthread.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <mem_map.h>
#include "arm_math.h"
#include "arm_nnfunctions.h"
#include "ref_functions.h"

#undef BSP_USING_NN_ACC

#define BLOCK_NNACC_SIZE        32768
#define NNACC_EXTRA_RAM_BASE    0x200E0000
#define TEST_SIGMOID
#define TEST_TANH
#define TEST_POOL
#define TEST_RELU
#define TEST_IP
#define TEST_CONV
#define TEST_NONSQUARE
#define TEST_NNMULT
//#define TEST_DS_CNN
#ifdef TEST_DS_CNN
    #include "ds_cnn.h"
    #include "kws.h"
#endif
#ifndef bool
    #define bool unsigned char
#endif
#ifndef false
    #define false 0
#endif

#ifndef true
    #define true 1
#endif


int test_index = 0;
q7_t test_flags[50];
bool test_pass;

q7_t     *test1;
q15_t    *test2;
q7_t     *test3;
q15_t    *test4;

void initialize_results_q7(q7_t *ref, q7_t *opt, int length)
{
    // arm_fill_q7(0, ref, length);
    memset(ref, 0, sizeof(q7_t)*length);
    // arm_fill_q7(37, opt, length);
    memset(opt, 0x25, sizeof(q7_t)*length);
}

void initialize_results_q15(q15_t *ref, q15_t *opt, int length)
{
    // arm_fill_q15(0, ref, length);
    memset(ref, 0, sizeof(q15_t)*length);
    // arm_fill_q15(0x5F5, opt, length);
    memset(opt, 0x5F5, sizeof(q15_t)*length);
}

void verify_results_q7(q7_t *ref, q7_t *opt, int length)
{

    bool      if_match = true;

    for (int i = 0; i < length; i++)
    {
        if (ref[i] != opt[i])
        {
            rt_kprintf("Output mismatch at %d, expected %d, actual %d\r\n", i, ref[i], opt[i]);
            if_match = false;
        }
    }

    if (if_match == true)
    {
        rt_kprintf("Outputs match.\r\n\r\n");
        test_flags[test_index++] = 0;
    }
    else
    {
        rt_kprintf("Output mismatch.\r\n\r\n");
        test_flags[test_index++] = 1;
    }

}

void verify_results_q15(q15_t *ref, q15_t *opt, int length)
{

    bool      if_match = true;

    for (int i = 0; i < length; i++)
    {
        if (ref[i] != opt[i])
        {
            rt_kprintf("Output mismatch at %d, expected %d, actual %d\r\n", i, ref[i], opt[i]);

            if_match = false;
        }
    }

    if (if_match == true)
    {
        rt_kprintf("Outputs match.\r\n\r\n");
        test_flags[test_index++] = 0;
    }
    else
    {
        test_flags[test_index++] = 1;
    }
}

void malloc_all(uint32_t size)
{
    test1 = malloc(sizeof(q7_t) * size);
    test2 = malloc(sizeof(q15_t) * size);
    test3 = malloc(sizeof(q7_t) * size);
    test4 = malloc(sizeof(q15_t) * size);
}
void free_all(void)
{
    if (test1)
    {
        free(test1);
        test1 = NULL;
    }
    if (test2)
    {
        free(test2);
        test2 = NULL;
    }
    if (test3)
    {
        free(test3);
        test3 = NULL;
    }
    if (test4)
    {
        free(test4);
        test4 = NULL;
    }
}

#ifdef TEST_NNMULT

void test_nnmult(int argc, char *argv[])
{
#define NNMULT_DIM 128
    int nnmult_dim = NNMULT_DIM;

    if (argc >= 3)
        nnmult_dim = atoi(argv[2]);

    malloc_all(nnmult_dim * 2);

    q7_t *mult_out_q7 = test3;
    q7_t *mult_ref_q7 = test3 + nnmult_dim;
    q15_t *mult_out_q15 = test4;
    q15_t *mult_ref_q15 = test4 + nnmult_dim;

    for (int i = 0; i < nnmult_dim * 2; i++)
    {
        test1[i] = (rand() % 256 - 128);
        test2[i] = (rand() % 65536 - 32768);
    }

    // Test q7
    arm_nn_mult_q7(test1, test1 + nnmult_dim, mult_out_q7, 5, nnmult_dim);

    arm_nn_mult_q7_ref(test1, test1 + nnmult_dim, mult_ref_q7, 5, nnmult_dim);

    verify_results_q7(mult_out_q7, mult_ref_q7, nnmult_dim);

    arm_nn_mult_q7(test1, test1 + nnmult_dim, mult_out_q7, 9, nnmult_dim);

    arm_nn_mult_q7_ref(test1, test1 + nnmult_dim, mult_ref_q7, 9, nnmult_dim);

    verify_results_q7(mult_out_q7, mult_ref_q7, nnmult_dim);

    // Test q15
    arm_nn_mult_q15(test2, test2 + nnmult_dim, mult_out_q15, 13, nnmult_dim);

    arm_nn_mult_q15_ref(test2, test2 + nnmult_dim, mult_ref_q15, 13, nnmult_dim);

    verify_results_q15(mult_out_q15, mult_ref_q15, nnmult_dim);

    arm_nn_mult_q15(test2, test2 + nnmult_dim, mult_out_q15, 18, nnmult_dim);

    arm_nn_mult_q15_ref(test2, test2 + nnmult_dim, mult_ref_q15, 18, nnmult_dim);

    verify_results_q15(mult_out_q15, mult_ref_q15, nnmult_dim);

    free_all();
}
#else
#define test_nnmult(argc,argv)
#endif

#ifdef TEST_SIGMOID
void test_sigmoid(int argc, char *argv[])
{
#define SIGMOID_DIM 128
    int sigmoid_dim = SIGMOID_DIM;

    if (argc >= 3)
        sigmoid_dim = atoi(argv[2]);

    /* This part tests the running of sigmoid functions */
    malloc_all(sigmoid_dim);
    srand(1);

    for (int i = 0; i < sigmoid_dim; i++)
    {
        test1[i] = (rand() % 256 - 128);
        test2[i] = (rand() % 65536 - 32768);
        test3[i] = test1[i];
        test4[i] = test2[i];
    }

    arm_nn_activations_direct_q7(test3, sigmoid_dim, 3, ARM_SIGMOID);

    for (int i = 0; i < sigmoid_dim; i++)
    {
        rt_kprintf("in: %d  out: %d\n", test1[i], test3[i]);
    }

    rt_kprintf("start testing q15_t sigmoid\n\n");

    arm_nn_activations_direct_q15(test4, sigmoid_dim, 3, ARM_SIGMOID);

    for (int i = 0; i < sigmoid_dim; i++)
    {
        rt_kprintf("in: %d  out: %d\n", test2[i], test4[i]);
    }

    free_all();

}
#else
#define test_sigmoid(argc,argv)
#endif


#ifdef TEST_TANH
void test_tanh(int argc, char *argv[])
{
#define TANH_DIM 128
    int tanh_dim = TANH_DIM; /* This part tests the running of sigmoid functions */

    if (argc >= 3)
        tanh_dim = atoi(argv[2]);

    malloc_all(tanh_dim);

    srand(1);

    for (int i = 0; i < tanh_dim; i++)
    {
        test1[i] = (rand() % 256 - 128);
        test2[i] = (rand() % 65536 - 32768);
        test3[i] = test1[i];
        test4[i] = test2[i];
    }

    arm_nn_activations_direct_q7(test3, tanh_dim, 3, ARM_TANH);

    rt_kprintf("start testing q7_t tanh\n\n");

    for (int i = 0; i < tanh_dim; i++)
    {
        rt_kprintf("in: %d  out: %d\n", test1[i], test3[i]);
    }

    rt_kprintf("start testing q15_t tanh\n\n");

    arm_nn_activations_direct_q15(test4, tanh_dim, 3, ARM_TANH);

    for (int i = 0; i < tanh_dim; i++)
    {
        rt_kprintf("in: %d  out: %d\n", test2[i], test4[i]);
    }

    free_all();
}
#else
#define test_tanh(argc,argv)
#endif

#ifdef TEST_POOL
void test_pool(int argc, char *argv[])
{
#define POOL_IM_DIM 32
#define POOL_IM_CH 8
    int pool_im_dim = POOL_IM_DIM;
    int pool_im_ch = POOL_IM_CH;

    if (argc >= 4)
    {
        pool_im_dim = atoi(argv[2]);
        pool_im_ch = atoi(argv[3]);
    }

    test1 = malloc(sizeof(q7_t) * (pool_im_dim * pool_im_dim * pool_im_ch * 2));
    test2 = malloc(sizeof(q15_t) * (pool_im_dim * pool_im_ch));
    test3 = malloc(sizeof(q7_t) * (pool_im_dim * pool_im_dim * pool_im_ch));

    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        test1[i] = (rand() % 256 - 128);
    }

    q7_t     *img_in = test1 + pool_im_dim * pool_im_dim * pool_im_ch;
    q7_t     *pool_out_ref = test3;
    q7_t     *pool_out_opt = test3 + pool_im_dim * pool_im_dim * pool_im_ch / 2;

    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        test3[i] = 0;
    }

    // copy over the img input
    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        img_in[i] = test1[i];
    }

    initialize_results_q7(pool_out_ref, pool_out_opt, pool_im_dim / 2 * pool_im_dim / 2 * pool_im_ch);

    rt_kprintf("Start maxpool reference implementation\n");

    arm_maxpool_q7_HWC_ref(img_in, pool_im_dim, pool_im_ch, 3, 0, 2, pool_im_dim / 2, (q7_t *) test2, pool_out_ref);

    // copy over the img input
    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        img_in[i] = test1[i];
    }

    rt_kprintf("Start maxpool opt implementation\n");

    arm_maxpool_q7_HWC(img_in, pool_im_dim, pool_im_ch, 3, 0, 2, pool_im_dim / 2, (q7_t *) test2, pool_out_opt);

    verify_results_q7(pool_out_ref, pool_out_opt, pool_im_dim / 2 * pool_im_dim / 2 * pool_im_ch);

    // copy over the img input
    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        img_in[i] = test1[i];
    }

    // copy over the img input
    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        img_in[i] = test1[i];
    }

    rt_kprintf("Start avepool ref implementation\n");

    arm_avepool_q7_HWC_ref(img_in, pool_im_dim, pool_im_ch, 3, 0, 2, pool_im_dim / 2, (q7_t *) test2, pool_out_ref);

    // copy over the img input
    for (int i = 0; i < pool_im_dim * pool_im_dim * pool_im_ch; i++)
    {
        img_in[i] = test1[i];
    }

    rt_kprintf("Start avepool opt implementation\n");

    arm_avepool_q7_HWC(img_in, pool_im_dim, pool_im_ch, 3, 0, 2, pool_im_dim / 2, (q7_t *) test2, pool_out_opt);

    // special check here
    bool      if_ave_pool_match = true;
    for (int i = 0; i < pool_im_dim / 2 * pool_im_dim / 2 * pool_im_ch; i++)
    {
        // we tolerate at most difference of 1 here because of rounding errors
        if (pool_out_ref[i] - pool_out_opt[i] >= 2 || pool_out_opt[i] - pool_out_ref[i] >= 2)
        {
            rt_kprintf("Output mismatch at %d, expected %d, actual %d\n", i, pool_out_ref[i], pool_out_opt[i]);
            if_ave_pool_match = false;
        }
    }
    if (if_ave_pool_match == true)
    {
        rt_kprintf("Outputs match.\n");
    }

    free_all();
}
#else
#define test_pool(argc,argv)
#endif

#ifdef TEST_RELU
void test_relu(int argc, char *argv[])
{
#define RELU_DIM 127
    int relu_dim = RELU_DIM;

    if (argc >= 3)
        relu_dim = atoi(argv[2]);

    malloc_all(relu_dim);

    for (int i = 0; i < relu_dim; i++)
    {
        test1[i] = (rand() % 256 - 128);
        test2[i] = (rand() % 65536 - 32768);
        test3[i] = test1[i];
        test4[i] = test2[i];
    }

    q7_t     *relu_ref_data_q7 = test1;
    q7_t     *relu_opt_data_q7 = test3;
    q15_t    *relu_ref_data_q15 = test2;
    q15_t    *relu_opt_data_q15 = test4;

    rt_kprintf("Start ref relu q7 implementation\n");

    arm_relu_q7_ref(relu_ref_data_q7, relu_dim);

    rt_kprintf("Start opt relu q7 implementation\n");

    arm_relu_q7(relu_opt_data_q7, relu_dim);

    verify_results_q7(relu_ref_data_q7, relu_opt_data_q7, relu_dim);

    rt_kprintf("Start ref relu q15 implementation\n");

    arm_relu_q15_ref(relu_ref_data_q15, relu_dim);

    rt_kprintf("Start opt relu q15 implementation\n");

    arm_relu_q15(relu_opt_data_q15, relu_dim);

    verify_results_q15(relu_ref_data_q15, relu_opt_data_q15, relu_dim);

    free_all();
}
#else
#define test_relu(argc,argv)
#endif

#ifdef TEST_IP
void test_ip(int argc, char *argv[])
{

#define IP_ROW_DIM 127
#define IP_COL_DIM 127

    int ip_row_dim = IP_ROW_DIM;
    int ip_col_dim = IP_COL_DIM;

    static q7_t      ip_weights[IP_ROW_DIM * IP_COL_DIM] __attribute__((aligned(4))) = IP2_WEIGHT;
    static q7_t      ip_q7_opt_weights[IP_ROW_DIM * IP_COL_DIM] __attribute__((aligned(4))) = IP4_WEIGHT;
    static q7_t      ip_q7_q15_opt_weights[IP_ROW_DIM * IP_COL_DIM] __attribute__((aligned(4))) = IP4_q7_q15_WEIGHT;
    static q15_t     ip_q15_weights[IP_ROW_DIM * IP_COL_DIM] __attribute__((aligned(4))) = IP2_WEIGHT;
    static q15_t     ip_q15_opt_weights[IP_ROW_DIM * IP_COL_DIM] __attribute__((aligned(4))) = IP4_WEIGHT_Q15;

    test1 = malloc(sizeof(q7_t) * (IP_COL_DIM + IP_ROW_DIM));
    test2 = malloc(sizeof(q15_t) * (IP_COL_DIM));
    test3 = malloc(sizeof(q7_t) * (IP_ROW_DIM * 3));
    test4 = malloc(sizeof(q15_t) * (IP_COL_DIM + IP_ROW_DIM * 2 + 1)); // Multiple of 4 bytes

    for (int i = 0; i < IP_ROW_DIM + IP_COL_DIM; i++)
    {
        test1[i] = rand() % 256 - 100;
    }
    for (int i = 0; i < IP_ROW_DIM * 3; i++)
    {
        test3[i] = 0;
    }

    q7_t     *ip_bias_q7 = test1 + IP_COL_DIM;

    q7_t     *ip_out_q7_ref = test3;
    q7_t     *ip_out_q7_opt = test3 + IP_ROW_DIM;
    q7_t     *ip_out_q7_opt_fast = test3 + 2 * IP_ROW_DIM;
    q15_t    *ip_out_q15_ref = test4 + IP_COL_DIM;
    q15_t    *ip_out_q15_opt = test4 + IP_COL_DIM + IP_ROW_DIM + 1; // Aligned to 4 bytes.

    initialize_results_q7(ip_out_q7_ref, ip_out_q7_opt, IP_ROW_DIM);
    initialize_results_q7(ip_out_q7_ref, ip_out_q7_opt_fast, IP_ROW_DIM);
    initialize_results_q7(ip_out_q7_ref, ip_out_q7_opt_fast, IP_ROW_DIM);

    rt_kprintf("Start ref q7 implementation\n");

    arm_fully_connected_q7_ref(test1, ip_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7, ip_out_q7_ref, test2);

    rt_kprintf("Start q7 implementation\n");

    arm_fully_connected_q7(test1, ip_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7, ip_out_q7_opt, test2);

    verify_results_q7(ip_out_q7_ref, ip_out_q7_opt, IP_ROW_DIM);

    rt_kprintf("Start q7 ref opt implementation\n");

    arm_fully_connected_q7_opt_ref(test1, ip_q7_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7,
                                   ip_out_q7_opt_fast, test2);

    verify_results_q7(ip_out_q7_ref, ip_out_q7_opt_fast, IP_ROW_DIM);

    rt_kprintf("Start q7 opt implementation\n");

    arm_fully_connected_q7_opt(test1, ip_q7_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7, ip_out_q7_opt_fast,
                               test2);

    verify_results_q7(ip_out_q7_ref, ip_out_q7_opt_fast, IP_ROW_DIM);

    for (int i = 0; i < IP_ROW_DIM + IP_COL_DIM; i++)
    {
        test4[i] = (rand() % 65536 - 32768);
    }

    initialize_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start ref q15 implementation\n");

    arm_fully_connected_q15_ref(test4, ip_q15_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, test2, ip_out_q15_ref, NULL);

    rt_kprintf("Start q15 implementation\n");

    arm_fully_connected_q15(test4, ip_q15_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, test2, ip_out_q15_opt, NULL);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start ref opt q15 implementation\n");

    arm_fully_connected_q15_opt_ref(test4, ip_q15_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, test2, ip_out_q15_opt,
                                    NULL);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start opt q15 implementation\n");

    arm_fully_connected_q15_opt(test4, ip_q15_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, test2, ip_out_q15_opt, NULL);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    initialize_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start ref q7_q15 implementation\n");

    arm_fully_connected_mat_q7_vec_q15_ref(test4, ip_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7, ip_out_q15_ref,
                                           test2);

    rt_kprintf("Start q7_q15 implementation\n");

    arm_fully_connected_mat_q7_vec_q15(test4, ip_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7, ip_out_q15_opt,
                                       test2);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start ref opt q7_q15 implementation\n");

    arm_fully_connected_mat_q7_vec_q15_opt_ref(test4, ip_q7_q15_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7,
            ip_out_q15_opt, test2);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    rt_kprintf("Start opt q7_q15 implementation\n");

    arm_fully_connected_mat_q7_vec_q15_opt(test4, ip_q7_q15_opt_weights, IP_COL_DIM, IP_ROW_DIM, 1, 7, ip_bias_q7,
                                           ip_out_q15_opt, test2);

    verify_results_q15(ip_out_q15_ref, ip_out_q15_opt, IP_ROW_DIM);

    free_all();

}
#else
#define test_ip(argc,argv)
#endif

#ifdef TEST_NONSQUARE
void test_nonsquare(int argc, char *argv[])
{
    /* Use RCONV to differential with square CONV */
    q15_t   *rconv_buf;

#define RCONV_IM_DIM_X 10
#define RCONV_IM_DIM_Y 8
#ifdef BSP_USING_NN_ACC     // When 1X1 nonsquare, input channel number must be 1.
#define RCONV_IM_CH 1
#else
#define RCONV_IM_CH 4
#endif
#define RCONV_KER_DIM_X 5
#define RCONV_KER_DIM_Y 3
#define RCONV_STRIDE_X 1
#define RCONV_STRIDE_Y 1
#define RCONV_PADDING_X 2
#define RCONV_PADDING_Y 1
#define RCONV_OUT_CH 4
#define RCONV_OUT_DIM_X 10
#define RCONV_OUT_DIM_Y 8

    q7_t    *rconv_weight_q7,  *rconv_bias_q7, *rconv_im_in_q7, *rconv_im_out_opt_q7, *rconv_im_out_ref_q7 ;

    test1 = malloc(sizeof(q7_t) * (RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH + RCONV_OUT_CH));
#ifdef BSP_USING_NN_ACC     // Input/output/weight/bias need to be in different 32K block when accelerator is run
    rconv_buf = (q15_t *)NNACC_EXTRA_RAM_BASE;
    rconv_im_in_q7 = (q7_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE);
    rconv_weight_q7 = test1;
    rconv_bias_q7 = (q7_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 2);
    rconv_im_out_ref_q7 = (q7_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 3);
    rconv_im_out_opt_q7 = rconv_im_out_ref_q7 + RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH;

    for (int i = 0; i < RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH; i++)
        rconv_weight_q7[i] = rand() % 256 - 100;
    for (int i = 0; i < RCONV_OUT_CH; i++)
        rconv_bias_q7[i] = rand() % 256 - 100;
    for (int i = 0; i < RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH; i++)
        rconv_im_in_q7[i] = rand() % 256 - 100;
#else
    test2 = malloc(sizeof(q15_t) * (2 * RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH));
    test3 = malloc(sizeof(q7_t) * (RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH + 2 * RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH));
    for (int i = 0; i < RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH + RCONV_OUT_CH; i++)
        test1[i] = rand() % 256 - 100;

    for (int i = 0;
            i < RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH + 2 * RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH; i++)
        test3[i] = rand() % 256 - 100;
    rconv_buf = test2;
    rconv_im_in_q7 = test3;
    rconv_weight_q7 = test1;
    rconv_bias_q7 = test1 + RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH;
    rconv_im_out_ref_q7 = test3 + RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH;
    rconv_im_out_opt_q7 = rconv_im_out_ref_q7 + RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH;
#endif



    initialize_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    rt_kprintf("start conv q7 nonsquare ref implementation\n");
    arm_convolve_HWC_q7_ref_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                      RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                      RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_ref_q7,
                                      RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    rt_kprintf("start conv q7 nonsquare opt implementation\n");
    rt_kprintf("in=%p, weight=%p, bias=%p, out=%p, rconv_buf=%p\n", rconv_im_in_q7, rconv_weight_q7, rconv_bias_q7, rconv_im_out_opt_q7, rconv_buf);
    arm_convolve_HWC_q7_fast_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                       RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                       RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_opt_q7,
                                       RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    verify_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    initialize_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    rt_kprintf("start conv q7 nonsquare ref implementation\n");
    arm_convolve_HWC_q7_ref_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                      RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                      RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_ref_q7,
                                      RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    rt_kprintf("start conv q7 nonsquare basic implementation, hardware accelerated \n");
    arm_convolve_HWC_q7_basic_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                        RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                        RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_opt_q7,
                                        RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    verify_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    initialize_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    rt_kprintf("start 1x1 conv q7 nonsquare fast implementation\n");
    arm_convolve_HWC_q7_fast_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                       RCONV_OUT_CH, 1, 1, 0, 0, RCONV_STRIDE_X,
                                       RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_ref_q7, RCONV_OUT_DIM_X,
                                       RCONV_OUT_DIM_Y, rconv_buf, NULL);

    rt_kprintf("start 1x1 conv q7 nonsquare dedicated function implementation, Hardware accelerated\n");
    arm_convolve_1x1_HWC_q7_fast_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q7,
                                           RCONV_OUT_CH, 1, 1, 0, 0, RCONV_STRIDE_X,
                                           RCONV_STRIDE_Y, rconv_bias_q7, 1, 7, rconv_im_out_opt_q7, RCONV_OUT_DIM_X,
                                           RCONV_OUT_DIM_Y, rconv_buf, NULL);

    verify_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    rt_kprintf("start depthwise separable conv q7 nonsquare ref implementation\n");
    arm_depthwise_separable_conv_HWC_q7_ref_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH,
            rconv_weight_q7, RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y,
            RCONV_PADDING_X, RCONV_PADDING_Y, RCONV_STRIDE_X, RCONV_STRIDE_Y,
            rconv_bias_q7, 1, 7, rconv_im_out_ref_q7, RCONV_OUT_DIM_X,
            RCONV_OUT_DIM_Y, rconv_buf, NULL);

    rt_kprintf("start depthwise separable conv q7 nonsquare opt implementation, Hardware accelerated\n");
    arm_depthwise_separable_conv_HWC_q7_nonsquare(rconv_im_in_q7, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH,
            rconv_weight_q7, RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y,
            RCONV_PADDING_X, RCONV_PADDING_Y, RCONV_STRIDE_X, RCONV_STRIDE_Y,
            rconv_bias_q7, 1, 7, rconv_im_out_opt_q7, RCONV_OUT_DIM_X,
            RCONV_OUT_DIM_Y, rconv_buf, NULL);

    verify_results_q7(rconv_im_out_ref_q7, rconv_im_out_opt_q7, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);
#ifdef BSP_USING_NN_ACC
    free(test1);
#else
    free_all();
#endif

    q15_t     *rconv_weight_q15, *rconv_bias_q15, *rconv_im_in_q15, *rconv_im_out_opt_q15, *rconv_im_out_ref_q15 ;

    test2 = malloc(sizeof(q15_t) * (RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH + RCONV_OUT_CH)); // weights + bias
#ifdef BSP_USING_NN_ACC     // Input/output/weight/bias need to be in different 32K block when accelerator is run
    rconv_weight_q15 = test2;
    rconv_buf = (q15_t *)NNACC_EXTRA_RAM_BASE;
    rconv_bias_q15 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE);
    rconv_im_in_q15 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 2);
    rconv_im_out_ref_q15 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 3);
    rconv_im_out_opt_q15 = rconv_im_out_ref_q15 + RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH;
    for (int i = 0; i < RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH; i++)
        rconv_weight_q15[i] = rand() % 65536 - 32768;
    for (int i = 0; i < RCONV_OUT_CH; i++)
        rconv_bias_q15[i] = rand() %  65536 - 32768;
    for (int i = 0; i < RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH; i++)
        rconv_im_in_q15[i] = rand() %  65536 - 32768;

#else
    test4 = malloc(sizeof(q15_t) * (2 * RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH //buffer
                                    + RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH + 2 * RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH)); // i/o
    for (int i = 0; i < RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH + RCONV_OUT_CH; i++)
    {
        test2[i] = rand() %  65536 - 32768;
    }
    for (int i = 0;
            i < 2 * RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH
            + RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH + 2 * RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH;
            i++)
    {
        test4[i] = rand() %  65536 - 32768;
    }
    rconv_buf = test4;
    rconv_weight_q15 = test2;
    rconv_bias_q15 = test2 + RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH * RCONV_OUT_CH;
    rconv_im_in_q15 = test4 + 2 * RCONV_KER_DIM_Y * RCONV_KER_DIM_X * RCONV_IM_CH;
    rconv_im_out_ref_q15 = rconv_im_in_q15 + RCONV_IM_DIM_Y * RCONV_IM_DIM_X * RCONV_IM_CH;
    rconv_im_out_opt_q15 = rconv_im_out_ref_q15 + RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH;
#endif


    initialize_results_q15(rconv_im_out_ref_q15, rconv_im_out_opt_q15, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

    rt_kprintf("start conv q15 nonsquare ref implementation\n");
    arm_convolve_HWC_q15_nonsquare_ref(rconv_im_in_q15, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q15,
                                       RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                       RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q15, 1, 7, rconv_im_out_ref_q15,
                                       RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    rt_kprintf("start conv q5 nonsquare opt implementation\n");
    arm_convolve_HWC_q15_fast_nonsquare(rconv_im_in_q15, RCONV_IM_DIM_X, RCONV_IM_DIM_Y, RCONV_IM_CH, rconv_weight_q15,
                                        RCONV_OUT_CH, RCONV_KER_DIM_X, RCONV_KER_DIM_Y, RCONV_PADDING_X, RCONV_PADDING_Y,
                                        RCONV_STRIDE_X, RCONV_STRIDE_Y, rconv_bias_q15, 1, 7, rconv_im_out_opt_q15,
                                        RCONV_OUT_DIM_X, RCONV_OUT_DIM_Y, rconv_buf, NULL);

    verify_results_q15(rconv_im_out_ref_q15, rconv_im_out_opt_q15, RCONV_OUT_DIM_Y * RCONV_OUT_DIM_X * RCONV_OUT_CH);

#ifdef BSP_USING_NN_ACC
    free(test2);
#else
    free_all();
#endif

}
#else
#define test_nonsquare(argc,argv)
#endif

#ifdef TEST_CONV
void test_conv(int argc, char *argv[])
{
#define CONV_IM_DIM 16
#define CONV_IM_CH 16
#define CONV_KER_DIM 5
#define CONV_OUT_CH 16
#define CONV_OUT_DIM 16
    int conv_im_dim = CONV_IM_DIM;
    int conv_im_ch = CONV_IM_CH;
    int conv_ker_dim = CONV_KER_DIM;
    int conv_out_ch = CONV_OUT_CH;
    int conv_out_dim = CONV_OUT_DIM;

    if (argc >= 7)
    {
        conv_im_dim = atoi(argv[2]);
        conv_im_ch = atoi(argv[3]);
        conv_ker_dim = atoi(argv[4]);
        conv_out_ch = atoi(argv[5]);
        conv_out_dim = atoi(argv[6]);
    }

    q7_t    *conv_weight_q7, *conv_bias_q7, *conv_im_in_q7, *conv_im_out_opt_q7, *conv_im_out_ref_q7;
    q15_t   *conv_buf;

    test1 = malloc(sizeof(q7_t) * (conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch + conv_out_ch));

#ifdef BSP_USING_NN_ACC     // Input/output/weight/bias need to be in different 32K block when accelerator is run
    test2 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE);
    test3 = (q7_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 2);
    test4 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 3);
    conv_buf = test2 + conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch;
    conv_weight_q7 = test1;
    conv_bias_q7 = (q7_t *)NNACC_EXTRA_RAM_BASE;
    conv_im_in_q7 = test3;
    conv_im_out_ref_q7 = (q7_t *)test4;
    conv_im_out_opt_q7 = conv_im_out_ref_q7 + conv_out_dim * conv_out_dim * conv_out_ch;
    for (int i = 0; i < conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch ; i++)
        conv_weight_q7[i] = rand() % 256 - 100;
    for (int i = 0; i < conv_out_ch; i++)
        conv_bias_q7[i] = rand() % 256 - 100;
    for (int i = 0; i < conv_im_dim * conv_im_dim * conv_im_ch ; i++)
        conv_im_in_q7[i] = rand() % 256 - 100;
#else
    for (int i = 0; i < conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch + conv_out_ch; i++)
    {
        test1[i] = rand() % 256 - 100;
    }
    test2 = malloc(sizeof(q15_t) * (conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch +
                                    2 * conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch + conv_out_ch));
    test3 = malloc(sizeof(q7_t) * (conv_im_dim * conv_im_dim * conv_im_ch + 2 * conv_out_dim * conv_out_dim * conv_out_ch));
    for (int i = 0; i < conv_im_dim * conv_im_dim * conv_im_ch + 2 * conv_out_dim * conv_out_dim * conv_out_ch; i++)
    {
        test3[i] = rand() % 256 - 100;
    }
    conv_buf = test2 + conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch;
    conv_weight_q7 = test1;
    conv_bias_q7 = test1 + conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch;
    conv_im_in_q7 = test3;
    conv_im_out_ref_q7 = test3 + conv_im_dim * conv_im_dim * conv_im_ch;
    conv_im_out_opt_q7 = conv_im_out_ref_q7 + conv_out_dim * conv_out_dim * conv_out_ch;
    test4 = malloc(sizeof(q15_t) * (conv_im_dim * conv_im_dim * conv_im_ch + 2 * conv_out_dim * conv_out_dim * conv_out_ch));
#endif
    for (int i = 0;
            i <
            conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch +
            2 * conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch + conv_out_ch; i++)
    {
        test2[i] = (rand() % 65536 - 32768);
    }

    initialize_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q7 ref implementation\n");

    arm_convolve_HWC_q7_ref(conv_im_in_q7, conv_im_dim, conv_im_ch, conv_weight_q7,
                            conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_ref_q7,
                            conv_out_dim, conv_buf, NULL);

    rt_kprintf("start q7 basic implementation\n");

    arm_convolve_HWC_q7_basic(conv_im_in_q7, conv_im_dim, conv_im_ch, conv_weight_q7,
                              conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_opt_q7,
                              conv_out_dim, conv_buf, NULL);

    verify_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q7 fast implementation\n");

    arm_convolve_HWC_q7_fast(conv_im_in_q7, conv_im_dim, conv_im_ch, conv_weight_q7,
                             conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_opt_q7,
                             conv_out_dim, conv_buf, NULL);

    verify_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    // testing with RGB
    rt_kprintf("start q7 ref implementation for RGB\n");

    arm_convolve_HWC_q7_ref(conv_im_in_q7, conv_im_dim, 3, conv_weight_q7,
                            conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_ref_q7,
                            conv_out_dim, conv_buf, NULL);

    rt_kprintf("start q7 basic implementation for RGB\n");

    arm_convolve_HWC_q7_basic(conv_im_in_q7, conv_im_dim, 3, conv_weight_q7,
                              conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_opt_q7,
                              conv_out_dim, conv_buf, NULL);

    verify_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q7 RGB implementation for RGB\n");

    arm_convolve_HWC_q7_RGB(conv_im_in_q7, conv_im_dim, 3, conv_weight_q7,
                            conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_opt_q7,
                            conv_out_dim, conv_buf, NULL);

    verify_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    // depthwise separable conv
    initialize_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q7 depthwise_separable_conv ref implementation\n");

    arm_depthwise_separable_conv_HWC_q7_ref(conv_im_in_q7, conv_im_dim, conv_im_ch, conv_weight_q7,
                                            conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_ref_q7,
                                            conv_out_dim, conv_buf, NULL);

    rt_kprintf("start q7 depthwise_separable_conv implementation\n");

    arm_depthwise_separable_conv_HWC_q7(conv_im_in_q7, conv_im_dim, conv_im_ch, conv_weight_q7,
                                        conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q7, 1, 7, conv_im_out_opt_q7,
                                        conv_out_dim, conv_buf, NULL);

    verify_results_q7(conv_im_out_ref_q7, conv_im_out_opt_q7, conv_out_dim * conv_out_dim * conv_out_ch);


    for (int i = 0; i < conv_im_dim * conv_im_dim * conv_im_ch + 2 * conv_out_dim * conv_out_dim * conv_out_ch; i++)
    {
        test4[i] = (rand() % 65536 - 32768);
    }

    q15_t    *conv_weight_q15, *conv_bias_q15, *conv_im_in_q15, *conv_im_out_opt_q15, *conv_im_out_ref_q15;
#ifdef BSP_USING_NN_ACC     // Input/output/weight/bias need to be in different 32K block when accelerator is run
    free(test1);
    test1 = malloc(sizeof(q15_t) * conv_out_ch);
    test2 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE);
    test3 = (q7_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 2);
    test4 = (q15_t *)(NNACC_EXTRA_RAM_BASE + BLOCK_NNACC_SIZE * 3);
    conv_weight_q15 = (q15_t *)NNACC_EXTRA_RAM_BASE;
    conv_bias_q15 = (q15_t *)test1;
    conv_buf = test2;
    conv_im_in_q15 = (q15_t *)test3;
    conv_im_out_ref_q15 = test4 + conv_im_dim * conv_im_dim * conv_im_ch;
    conv_im_out_opt_q15 = conv_im_out_ref_q15 + conv_out_dim * conv_out_dim * conv_out_ch;

    for (int i = 0; i < conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch; i++)
        conv_weight_q15[i] = (rand() % 65536 - 32768);
    for (int i = 0; i < conv_out_ch; i++)
        conv_bias_q15[i] = (rand() % 65536 - 32768);
    for (int i = 0; i < conv_im_dim * conv_im_dim * conv_im_ch; i++)
        conv_im_in_q15[i] = (rand() % 65536 - 32768);
#else
    conv_weight_q15 = test2;
    conv_buf = test2 + conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch;
    conv_bias_q15 =
        test2 + conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch +
        2 * conv_ker_dim * conv_ker_dim * conv_im_ch * conv_out_ch;
    conv_im_in_q15 = test4;
    conv_im_out_ref_q15 = test4 + conv_im_dim * conv_im_dim * conv_im_ch;
    conv_im_out_opt_q15 = conv_im_out_ref_q15 + conv_out_dim * conv_out_dim * conv_out_ch;
#endif

    // testing q15
    initialize_results_q15(conv_im_out_ref_q15, conv_im_out_opt_q15, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q15 ref implementation\n");

    arm_convolve_HWC_q15_ref(conv_im_in_q15, conv_im_dim, conv_im_ch, conv_weight_q15,
                             conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q15, 0, 15, conv_im_out_ref_q15,
                             conv_out_dim, conv_buf, NULL);

    rt_kprintf("start q15 basic implementation\n");

    arm_convolve_HWC_q15_basic(conv_im_in_q15, conv_im_dim, conv_im_ch, conv_weight_q15,
                               conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q15, 0, 15, conv_im_out_opt_q15,
                               conv_out_dim, conv_buf, NULL);

    verify_results_q15(conv_im_out_ref_q15, conv_im_out_opt_q15, conv_out_dim * conv_out_dim * conv_out_ch);

    rt_kprintf("start q15 fast implementation\n");

    arm_convolve_HWC_q15_fast(conv_im_in_q15, conv_im_dim, conv_im_ch, conv_weight_q15,
                              conv_out_ch, conv_ker_dim, 2, 1, conv_bias_q15, 0, 15, conv_im_out_opt_q15,
                              conv_out_dim, conv_buf, NULL);

    verify_results_q15(conv_im_out_ref_q15, conv_im_out_opt_q15, conv_out_dim * conv_out_dim * conv_out_ch);

#ifdef BSP_USING_NN_ACC
    free(test1);
#else
    free_all();
#endif

}
#else
#define test_conv(argc,argv)
#endif

#ifdef TEST_DS_CNN
extern kws_t *kws;
extern void ds_cnn_run_nn_ref(nn_t *handle, q7_t *in_data, q7_t *out_data);

void test_ds_cnn(int argc, char *argv[])
{
    int i;
    int loop = 1;

    if (argc >= 3)
        loop = atoi(argv[2]);
    while (loop > 0)
    {
        rt_kprintf("Test count down %d\n", loop);
        test1 = malloc(CONV1_IN_X * CONV1_IN_Y * 1);
        test3 = malloc(kws->nn->num_out_classes * 2);
        q7_t *opt_result = test3;
        q7_t *ref_result = test3 + kws->nn->num_out_classes * 1;
        for (int i = 0; i < CONV1_IN_X * CONV1_IN_Y * 1; i++)
        {
            test1[i] = rand() % 256 - 128;
        }
#ifdef TEST_VERBOSE
        rt_kprintf("\nInput:\n");
        rt_print_data((char *)test1, 0, CONV1_IN_X * CONV1_IN_Y * 1);
#endif
        initialize_results_q7(opt_result, ref_result, kws->nn->num_out_classes * 1);
        ds_cnn_run_nn(kws->nn, test1, opt_result);
        ds_cnn_run_nn_ref(kws->nn, test1, ref_result);
#ifdef TEST_VERBOSE
        rt_kprintf("\nTest result:\n");
        rt_print_data((char *)opt_result, 0, kws->nn->num_out_classes * 1);
#endif
        verify_results_q7(opt_result, ref_result, kws->nn->num_out_classes * 1);
        free_all();
        loop--;
    }
}
#else
#define test_ds_cnn(argc,argv)
#endif

int cmd_nntest(int argc, char *argv[])
{
    rt_kprintf("start tests\n");
    srand(rt_tick_get());

    // common pointers for testing data

    for (test_index = 0; test_index < 50; test_index++)
    {
        test_flags[test_index] = -1;
    }
    test_index = 0;

    if (strcmp(argv[1], "nnmult") == 0)
    {
        test_nnmult(argc, argv);
    }
    if (strcmp(argv[1], "sigmoid") == 0)
    {
        test_sigmoid(argc, argv);
    }
    if (strcmp(argv[1], "tanh") == 0)
    {
        test_tanh(argc, argv);
    }
    if (strcmp(argv[1], "pool") == 0)
    {
        test_pool(argc, argv);
    }
    if (strcmp(argv[1], "relu") == 0)
    {
        test_relu(argc, argv);
    }
    if (strcmp(argv[1], "ip") == 0)
    {
        test_ip(argc, argv);
    }
    if (strcmp(argv[1], "nonsqure") == 0)
    {
        test_nonsquare(argc, argv);
    }
    if (strcmp(argv[1], "conv") == 0)
    {
        test_conv(argc, argv);
    }
    if (strcmp(argv[1], "all") == 0)
    {
        test_nnmult(argc, argv);
        test_sigmoid(argc, argv);
        test_tanh(argc, argv);
        test_pool(argc, argv);
        test_relu(argc, argv);
        //test_ip(argc, argv);
        test_conv(argc, argv);
        test_nonsquare(argc, argv);
    }
    if (strcmp(argv[1], "ds_cnn") == 0)
    {
        test_ds_cnn(argc, argv);
    }
    test_pass = true;
    test_index = 0;
    while (test_flags[test_index] != -1)
    {
        if (test_flags[test_index])
        {
            test_pass = false;
        }
        test_index ++;
    }
    if (test_pass)
    {
        rt_kprintf("All tests passed\n");
    }
    else
    {
        rt_kprintf("Test failed passed\n");
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_nntest, __cmd_nntest, NN Accelerator test.);

int nn_test_run(void)
{
    char argv[][16] =
    {
        "nntest",
        "all"
    };
    cmd_nntest(2, (char **)argv);
    return RT_EOK;
}
//INIT_APP_EXPORT(nn_test_run);


