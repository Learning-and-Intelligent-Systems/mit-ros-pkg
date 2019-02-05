/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/* Vector addition: C = A + B.
 *
 * This sample is a very basic sample that implements element by element
 * vector addition. It is the same as the sample illustrating Chapter 3
 * of the programming guide with some additions like error checking.
 *
 */

// Includes
#include <stdio.h>
#include <sys/time.h>
// #include <cutil_inline.h>



timeval g_tick(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv;
}

double g_tock(timeval tprev)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
}

struct pt{
	float x,y,z;



};

// Variables
float* h_A;
float* h_B;
float* h_C;
float* d_A;
float* d_B;
float* d_C;
bool noprompt = false;

// Functions
void Cleanup(void);
void RandomInit(float*, int);
void ParseArguments(int, char**);

// Device code
__global__ void VecAdd(const float* A, const float* B, float* C, int N)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i < N)
        C[i] = A[i] + B[i];
}


// Device code
// Finds distance to the point
__global__ void PtDist(const float* A, const float* B, float* C, int N)
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i < N)
        C[i] = A[i] + B[i];
}

float * tocuda(float *hdata, size_t size){
	float * fptr;
	cudaMalloc((void**)&fptr, size);
    cudaMemcpy(fptr, hdata, size, cudaMemcpyHostToDevice) ;
	return fptr;
}


// Host code
int main(int argc, char** argv)
{
    printf("Vector addition\n");
    int N = 50000;
    size_t size = N * sizeof(float);
    ParseArguments(argc, argv);

    // Allocate input vectors h_A and h_B in host memory
    h_A = (float*)malloc(size);
    if (h_A == 0) Cleanup();
    h_B = (float*)malloc(size);
    if (h_B == 0) Cleanup();
    h_C = (float*)malloc(size);
    if (h_C == 0) Cleanup();
    
    // Initialize input vectors
    RandomInit(h_A, N);
    RandomInit(h_B, N);

    // Allocate vectors in device memory
     cudaMalloc((void**)&d_C, size) ;

    // Copy vectors from host memory to device memory
     d_B=tocuda(h_B,size);
     d_A=tocuda(h_A,size);

    // Invoke kernel
    int threadsPerBlock = 256;
    int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;
    VecAdd<<<blocksPerGrid, threadsPerBlock>>>(d_A, d_B, d_C, N);
    //cutilCheckMsg("kernel launch failure");
#ifdef _DEBUG
     cudaThreadSynchronize() );
#endif

    // Copy result from device memory to host memory
    // h_C contains the result in host memory
     cudaMemcpy(h_C, d_C, size, cudaMemcpyDeviceToHost) ;
    
    // Verify result
    int i;
    for (i = 0; i < N; ++i) {
        float sum = h_A[i] + h_B[i];
        if (fabs(h_C[i] - sum) > 1e-5)
            break;
    }
    printf("%s \n", (i == N) ? "PASSED" : "FAILED");
    
    Cleanup();
}

void Cleanup(void)
{
    // Free device memory
    if (d_A)
        cudaFree(d_A);
    if (d_B)
        cudaFree(d_B);
    if (d_C)
        cudaFree(d_C);

    // Free host memory
    if (h_A)
        free(h_A);
    if (h_B)
        free(h_B);
    if (h_C)
        free(h_C);
        
     cudaThreadExit() ;
    
    if (!noprompt) {
        printf("\nPress ENTER to exit...\n");
        fflush( stdout);
        fflush( stderr);
        getchar();
    }

    exit(0);
}

// Allocates an array with random float entries.
void RandomInit(float* data, int n)
{
    for (int i = 0; i < n; ++i)
        data[i] = rand() / (float)RAND_MAX;
}

// Parse program arguments
void ParseArguments(int argc, char** argv)
{
    for (int i = 0; i < argc; ++i)
        if (strcmp(argv[i], "--noprompt") == 0 ||
			strcmp(argv[i], "-noprompt") == 0) 
		{
            noprompt = true;
            break;
        }
}
