/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>



#include <blasfeo.h>

#include "benchmark_x_common.h"



#if defined(EXTERNAL_BLAS_NETLIB)
//#include "cblas.h"
//#include "lapacke.h"
#include "../include/d_blas.h"
#endif

#if defined(EXTERNAL_BLAS_OPENBLAS)
void openblas_set_num_threads(int num_threads);
//#include "cblas.h"
//#include "lapacke.h"
#include "../include/d_blas.h"
#endif

#if defined(EXTERNAL_BLAS_BLIS)
//void omp_set_num_threads(int num_threads);
#include "blis.h"
#endif

#if defined(EXTERNAL_BLAS_MKL)
#include "mkl.h"
#endif

#if defined(EXTERNAL_BLAS_ARMPL)
#include "armpl.h"
#endif



//#define PRINT_TO_FILE



int main()
	{

#if defined(EXTERNAL_BLAS_OPENBLAS)
openblas_set_num_threads(1);
#endif



// initialize blasfeo (e.g. pre-allocate memory buffers) (optional, not thread safe)
//blasfeo_init();



#if !defined(BLAS_API)
	printf("\nRecompile with BLAS_API=1 to run this benchmark!\n\n");
	return 0;
#endif

	printf("\n");
	printf("\n");
	printf("\n");

	printf("BLASFEO performance test - BLAS API - double precision\n");
	printf("\n");

	// maximum frequency of the processor
	const float GHz_max = GHZ_MAX;
	printf("Frequency used to compute theoretical peak: %5.1f GHz (edit benchmarks/cpu_freq.h to modify this value).\n", GHz_max);
	printf("\n");

	// maximum flops per cycle, double precision
#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	const float flops_max = 32;
	printf("Testing BLASFEO version for AVX512F instruction set, 64 bit (optimized for Intel Skylake-X): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_HASWELL)
	const float flops_max = 16;
	printf("Testing BLAS version for AVX2 and FMA instruction sets, 64 bit (optimized for Intel Haswell): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_SANDY_BRIDGE)
	const float flops_max = 8;
	printf("Testing BLAS version for AVX instruction set, 64 bit (optimized for Intel Sandy Bridge): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_INTEL_CORE)
	const float flops_max = 4;
	printf("Testing BLAS version for SSE3 instruction set, 64 bit (optimized for Intel Core): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X64_AMD_BULLDOZER)
	const float flops_max = 8;
	printf("Testing BLAS version for SSE3 and FMA instruction set, 64 bit (optimized for AMD Bulldozer): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_JAGUAR)
	const float flops_max = 2;
	printf("Testing BLAS version for AVX instruction set, 32 bit (optimized for AMD Jaguar): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_X86_AMD_BARCELONA)
	const float flops_max = 4; // 2 on jaguar
	printf("Testing BLAS version for SSE3 instruction set, 32 bit (optimized for AMD Barcelona): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_APPLE_M1)
	const float flops_max = 16;
	printf("Testing BLASFEO version for NEONv2 instruction set, 64 bit (optimized for Apple M1): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A76)
	const float flops_max = 8;
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A76): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A57)
	const float flops_max = 4;
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A57): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV8A_ARM_CORTEX_A53)
	const float flops_max = 4;
	printf("Testing BLAS version for NEONv2 instruction set, 64 bit (optimized for ARM Cortex A53): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A15)
	const float flops_max = 2;
	printf("Testing BLAS version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A15): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A7)
	const float flops_max = 0.5;
	printf("Testing BLAS version for VFPv4 instruction set, 32 bit (optimized for ARM Cortex A7): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_ARMV7A_ARM_CORTEX_A9)
	const float flops_max = 1;
	printf("Testing BLAS version for VFPv3 instruction set, 32 bit (optimized for ARM Cortex A9): theoretical peak %5.1f Gflops\n", flops_max*GHz_max);
#elif defined(TARGET_GENERIC)
	const float flops_max = 2;
	printf("Testing BLAS version for generic scalar instruction set: theoretical peak %5.1f Gflops ???\n", flops_max*GHz_max);
#endif



#ifdef PRINT_TO_FILE
	FILE *f;
	f = fopen("./build/benchmark_one.m", "w"); // a

	fprintf(f, "A = [%f %f];\n", GHz_max, flops_max);
	fprintf(f, "\n");
	fprintf(f, "B = [\n");
#endif

	printf("\nn\t Gflops\t    %%\t    time\t Gflops\t    %%\t    time\n\n");


	int ii, jj, ll;

	int rep, rep_in;
	int nrep_in = 4; // number of benchmark batches

	int nn[] = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 348, 352, 356, 360, 364, 368, 372, 376, 380, 384, 388, 392, 396, 400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444, 448, 452, 456, 460, 464, 468, 472, 476, 480, 484, 488, 492, 496, 500, 520, 540, 560, 580, 600, 620, 640, 680, 700, 720, 740, 760, 780, 800, 820, 840, 860, 880, 900, 920, 940, 960, 980, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000};
	int nnrep[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 400, 400, 400, 400, 400, 200, 200, 200, 200, 200, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 20, 20, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

//	for(ll=0; ll<1; ll++)
//	for(ll=0; ll<2; ll++) // up to 8
//	for(ll=0; ll<3; ll++) // up to 12
//	for(ll=0; ll<4; ll++) // up to 16
//	for(ll=0; ll<24; ll++)
//	for(ll=0; ll<63; ll++) // up to 256
	for(ll=0; ll<75; ll++) // up to 300
//	for(ll=0; ll<149; ll++) // up to 1000
//	for(ll=0; ll<159; ll++) // up to 2000
//	for(ll=0; ll<169; ll++) // up to 3000

		{

		int n = nn[ll];
//		int n = 12;
		int nrep = nnrep[ll]; ///8;
//		nrep = 10;
		nrep = nrep>1 ? nrep : 1;
//		int n = ll+1;
//		int nrep = nnrep[0];
//		n = n<64 ? 64 : n;
//		n = n<48 ? 48 : n;
//		n = n<24 ? 24 : n;
//		n = n<12 ? 12 : n;
//		n = n<8 ? 8 : n;
//		n = n<4 ? 4 : n;
		n = n<1 ? 1 : n;
//		nrep = 10000;
//		n = 32;

//		omp_set_num_threads(4);
		double *A; d_zeros_align(&A, n, n);
		for(ii=0; ii<n*n; ii++)
			A[ii] = ii;
		int lda = n;
//		d_print_mat(n, n, A, n);

		double *B; d_zeros_align(&B, n, n);
		for(ii=0; ii<n*n; ii++)
			B[ii] = 0;
		for(ii=0; ii<n; ii++)
			B[ii*(n+1)] = 1.0;
		int ldb = n;
//		d_print_mat(n, n, B, ldb);

		double *C; d_zeros_align(&C, n, n);
		for(ii=0; ii<n*n; ii++)
			C[ii] = -1;
		int ldc = n;
//		d_print_mat(n, n, C, ldc);

		double *D; d_zeros_align(&D, n, n);
		for(ii=0; ii<n*n; ii++)
			D[ii] = -1;
		int ldd = n;
//		d_print_mat(n, n, C, ldc);

		int *ipiv = malloc(n*sizeof(int));

		int bs = 4;

		struct blasfeo_dmat sA; blasfeo_allocate_dmat(n, n, &sA);
		blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);
		struct blasfeo_dmat sB; blasfeo_allocate_dmat(n, n, &sB);
		blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);
		struct blasfeo_dmat sC; blasfeo_allocate_dmat(n, n, &sC);
		blasfeo_pack_dmat(n, n, C, n, &sC, 0, 0);
		struct blasfeo_dmat sD; blasfeo_allocate_dmat(n, n, &sD);
		blasfeo_pack_dmat(n, n, D, n, &sD, 0, 0);


		/* timing */
		blasfeo_timer timer;

		double time_blasfeo   = 1e15;
		double time_blas      = 1e15;
		double time_blas_api  = 1e15;
		double tmp_time_blasfeo;
		double tmp_time_blas;
		double tmp_time_blas_api;

		/* benchmarks */

		char ta = 'n';
		char tb = 't';
		char uplo = 'u';
		int info = 0;

		double alpha = 1.0;
		double beta = 1.0;

		char c_l = 'l';
		char c_n = 'n';
		char c_r = 'r';
		char c_t = 't';
		char c_u = 'u';

		int fix_m, fix_n, fix_k;


#if 0
		int n2 = n*n;
		int i_1 = 1;
//		d_print_mat(n, n, B, n);
		char c_v = 'v';
//		char c_v = 'n';
		double *lambda = malloc(n*sizeof(double));
		double tmp_work = 0;
		int tmp_iwork = 0;
		int lwork = -1;
		int liwork = -1;
//		blasfeo_lapack_dsyevd(&c_v, &c_l, &n, B, &n, lambda, &tmp_work, &lwork, &tmp_iwork, &liwork, &info);
		dsyevd_(&c_v, &c_l, &n, B, &n, lambda, &tmp_work, &lwork, &tmp_iwork, &liwork, &info);
		lwork = tmp_work;
		lwork = (32+2)*n > lwork ? (32+2)*n : lwork; // optimal work for dsytrd
		lwork += 4200; // extra memory for dormqr
		liwork = tmp_iwork;
//		printf("\ndsyevd lwork %d liwork %d\n", lwork, liwork);
		double *work = malloc(lwork*sizeof(double));
		int *iwork = malloc(liwork*sizeof(int));
		dcopy_(&n2, B, &i_1, C, &i_1);
//		dsyrk_(&c_l, &c_t, &n, &n, &alpha, A, &n, &beta, C, &n);
		for(ii=0; ii<n; ii++)
			C[ii+n*ii] = 2.0;
		for(ii=0; ii<n-1; ii++)
			C[ii+1+n*ii] = 1.0;
		for(ii=0; ii<n-2; ii++)
			C[ii+2+n*ii] = 0.5;
		for(ii=0; ii<n-3; ii++)
			C[ii+3+n*ii] = 0.25;
#endif



#if 1
		/* call blas */
		for(rep_in=0; rep_in<nrep_in; rep_in++)
			{

			#if defined(EXTERNAL_BLAS_OPENBLAS) || defined(EXTERNAL_BLAS_NETLIB) || defined(EXTERNAL_BLAS_MKL) || defined(EXTERNAL_BLAS_SYSTEM)

//			for(ii=0; ii<n*n; ii++) C[ii] = B[ii];
//			dgemm_(&ta, &tb, &n, &n, &n, &alpha, A, &n, A, &n, &beta, C, &n);

			#endif

			// BENCHMARK_BLAS
			blasfeo_tic(&timer);

			// averaged repetions
			for(rep=0; rep<nrep; rep++)
				{

				#if defined(EXTERNAL_BLAS_OPENBLAS) || defined(EXTERNAL_BLAS_NETLIB) || defined(EXTERNAL_BLAS_MKL) || defined(EXTERNAL_BLAS_SYSTEM)

				dgemm_(&c_n, &c_n, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);

//				dtrsm_(&c_r, &c_l, &c_t, &c_n, &n, &n, &alpha, B, &n, C, &n);

//				dtrmm_(&c_r, &c_l, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);

//				for(ii=0; ii<n*n; ii++) C[ii] = B[ii];
//				dgemm_(&ta, &tb, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				for(ii=0; ii<n*n; ii++) D[ii] = C[ii];
//				dpotrf_(&uplo, &n, D, &n, &info);
//				dpotrf_(&uplo, &n, B, &n, &info);

				#endif

				}

			tmp_time_blas = blasfeo_toc(&timer) / nrep;
			time_blas = tmp_time_blas<time_blas ? tmp_time_blas : time_blas;
			// BENCHMARK_BLAS

			}
#endif

//		d_print_mat(n, n, C, ldc);
//		d_print_mat(n, n, D, ldd);





#if 1
		/* call blas with packing */
		for(rep_in=0; rep_in<nrep_in; rep_in++)
			{

//			for(ii=0; ii<n*n; ii++) C[ii] = B[ii];
//			blasfeo_dgemm(&ta, &tb, &n, &n, &n, &alpha, A, &n, A, &n, &beta, C, &n);

			fix_m = n;
			fix_n = n;
			fix_k = n;

			// BENCHMARK_BLASFEO
			blasfeo_tic(&timer);

			// averaged repetions
			for(rep=0; rep<nrep; rep++)
				{

//				#define blasfeo_blas_dgemm dgemm_

//				blasfeo_blas_dgemm(&c_n, &c_n, &fix_m, &fix_n, &fix_k, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_n, &c_t, &fix_m, &fix_n, &fix_k, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_t, &c_n, &fix_m, &fix_n, &fix_k, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_t, &c_t, &fix_m, &fix_n, &fix_k, &alpha, A, &n, B, &n, &beta, C, &n);

				blasfeo_blas_dgemm(&c_n, &c_n, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_n, &c_t, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_t, &c_n, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				blasfeo_blas_dgemm(&c_t, &c_t, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);

//				blasfeo_blas_dsyrk(&c_l, &c_n, &n, &n, &alpha, A, &n, &beta, C, &n);
//				blasfeo_blas_dsyrk(&c_l, &c_t, &n, &n, &alpha, A, &n, &beta, C, &n);
//				blasfeo_blas_dsyrk(&c_u, &c_n, &n, &n, &alpha, A, &n, &beta, C, &n);
//				blasfeo_blas_dsyrk(&c_u, &c_t, &n, &n, &alpha, A, &n, &beta, C, &n);

//				blasfeo_blas_dtrsm(&c_l, &c_l, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_l, &c_n, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_l, &c_t, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_l, &c_t, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_u, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_u, &c_n, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_u, &c_t, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_l, &c_u, &c_t, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_l, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_l, &c_n, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_l, &c_t, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_l, &c_t, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_u, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_u, &c_n, &c_u, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_u, &c_t, &c_n, &n, &n, &alpha, B, &n, C, &n);
//				blasfeo_blas_dtrsm(&c_r, &c_u, &c_t, &c_u, &n, &n, &alpha, B, &n, C, &n);

//				blasfeo_blas_dtrmm(&c_r, &c_l, &c_n, &c_n, &n, &n, &alpha, B, &n, C, &n);

//				blasfeo_lapack_dpotrf(&c_l, &n, B, &n, &info);
//				blasfeo_lapack_dpotrf(&c_u, &n, B, &n, &info);

//				blasfeo_lapack_dgetrf(&n, &n, B, &n, ipiv, &info);



//				for(ii=0; ii<n*n; ii++) C[ii] = B[ii];
//				blasfeo_blas_dgemm(&ta, &tb, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				for(ii=0; ii<n*n; ii++) D[ii] = C[ii];
//				blasfeo_lapack_dpotrf(&uplo, &n, D, &n);
//				blasfeo_lapack_dpotrf(&uplo, &n, B, &n, &info);

//				dgemm_(&c_n, &c_n, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				dgemm_(&c_n, &c_t, &n, &n, &n, &alpha, A, &n, B, &n, &beta, C, &n);
//				dpotrf_(&c_l, &n, B, &n, &info);

#if 0
				int memsize_A = blasfeo_memsize_dmat(n, n);
				int memsize_B = blasfeo_memsize_dmat(n, n);
				int memsize_C = blasfeo_memsize_dmat(n, n);

				int memsize = 64+memsize_A+memsize_B+memsize_C;

				void *mem = calloc(memsize, 1);
				void *mem_align = (void *) ( ( ( (unsigned long long) mem ) + 63) / 64 * 64 );

				struct blasfeo_dmat sA, sB, sC;

				blasfeo_create_dmat(n, n, &sA, mem_align);
				blasfeo_create_dmat(n, n, &sB, mem_align+memsize_A);
				blasfeo_create_dmat(n, n, &sC, mem_align+memsize_A+memsize_B);

				blasfeo_pack_dmat(n, n, A, n, &sA, 0, 0);
//				blasfeo_pack_dmat(n, n, B, n, &sB, 0, 0);
				blasfeo_pack_tran_dmat(n, n, B, n, &sB, 0, 0);
				blasfeo_pack_dmat(n, n, C, n, &sC, 0, 0);

				blasfeo_dgemm_nt(n, n, n, 1.0, &sA, 0, 0, &sB, 0, 0, 0.0, &sC, 0, 0, &sC, 0, 0);

				blasfeo_unpack_dmat(n, n, &sC, 0, 0, C, n);

				free(mem);
#endif

#if 0
				dcopy_(&n2, C, &i_1, B, &i_1);
//				blasfeo_lapack_dsyevd(&c_v, &c_l, &n, B, &n, lambda, work, &lwork, iwork, &liwork, &info);
				dsyevd_(&c_v, &c_l, &n, B, &n, lambda, work, &lwork, iwork, &liwork, &info);
//				d_print_mat(n, n, B, n);
//				d_print_mat(1, n, lambda, 1);
#endif
				}

			tmp_time_blas_api = blasfeo_toc(&timer) / nrep;
			time_blas_api = tmp_time_blas_api<time_blas_api ? tmp_time_blas_api : time_blas_api;
			// BENCHMARK_BLASFEO_END

			}
#endif

//		d_print_mat(n, n, C, ldc);
//		d_print_mat(n, n, D, ldd);





#if 1
		/* call blasfeo */
		for(rep_in=0; rep_in<nrep_in; rep_in++)
			{

			// BENCHMARK_BLASFEO
			blasfeo_tic(&timer);

			// averaged repetions
			for(rep=0; rep<nrep; rep++)
				{
				
//				blasfeo_dgemm_nn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dgemm_nt(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dgemm_tn(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dgemm_tt(n, n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dsyrk_ln(n, n, alpha, &sA, 0, 0, &sB, 0, 0, beta, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dpotrf_l(n, &sB, 0, 0, &sB, 0, 0);
//				blasfeo_dtrsm_rltn(n, n, alpha, &sB, 0, 0, &sC, 0, 0, &sC, 0, 0);
//				blasfeo_dtrmm_rlnn(n, n, alpha, &sB, 0, 0, &sC, 0, 0, &sC, 0, 0);

				}

			tmp_time_blasfeo = blasfeo_toc(&timer) / nrep;
			time_blasfeo = tmp_time_blasfeo<time_blasfeo ? tmp_time_blasfeo : time_blasfeo;
			// BENCHMARK_BLASFEO_END

			}
#endif

//		d_print_mat(n, n, C, ldc);
//		blasfeo_print_dmat(n, n, &sC, 0, 0);



		double Gflops_max = flops_max * GHz_max;

//		double flop_operation = 2.0*fix_m*fix_n*fix_k; // gemm

		double flop_operation = 2.0*n*n*n; // gemm
//		double flop_operation = 1.0*n*n*n; // syrk trsm
//		double flop_operation = 1.0/3.0*n*n*n; // potrf
//		double flop_operation = 2.0/3.0*n*n*n; // getrf

//		double flop_operation = 9.0*n*n*n; // syevd (v)
//		double flop_operation = 4.0/3.0*n*n*n; // syevd (n)

		double Gflops_blas      = 1e-9*flop_operation/time_blas;
		double Gflops_blas_api  = 1e-9*flop_operation/time_blas_api;
		double Gflops_blasfeo   = 1e-9*flop_operation/time_blasfeo;

		printf("%d\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\n",
			n,
			Gflops_blas_api, 100.0*Gflops_blas_api/Gflops_max, time_blas_api,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max, time_blas,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max, time_blasfeo);
#ifdef PRINT_TO_FILE
		fprintf(f, "%d\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\t%7.3f\t%7.3f\t%5.3e\n",
			n,
			Gflops_blas_api, 100.0*Gflops_blas_api/Gflops_max, time_blas_api,
			Gflops_blas, 100.0*Gflops_blas/Gflops_max, time_blas,
			Gflops_blasfeo, 100.0*Gflops_blasfeo/Gflops_max, time_blasfeo);
#endif

		d_free_align(A);
		d_free_align(B);
		d_free_align(C);
		d_free_align(D);
		free(ipiv);
		blasfeo_free_dmat(&sA);
		blasfeo_free_dmat(&sB);
		blasfeo_free_dmat(&sC);
		blasfeo_free_dmat(&sD);

#if 0
		free(lambda);
		free(work);
		free(iwork);
#endif
		}

	printf("\n");
#ifdef PRINT_TO_FILE
	fprintf(f, "];\n");

	fclose(f);
#endif



// quit blasfeo (e.g. free pre-allocated memory buffers)
//blasfeo_quit();



	return 0;

	}
