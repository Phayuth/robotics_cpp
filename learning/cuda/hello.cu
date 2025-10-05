#include <stdio.h>

__global__ void cuda_hello(void) {
    printf("Hello World from GPU! on Thread ID:%d \n", threadIdx.x);
}

int main(void) {
    cuda_hello<<<1, 10>>>();
    cudaDeviceReset();
    return 0;
}

/*The qualifier __global__ tells the compiler that the function will be called from
the CPU and exe- cuted on the GPU. */

/*Triple angle brackets mark a call from the host thread to the code on the device
side. A kernel is executed by an array of threads and all threads run the same
code. The parameters within the triple angle brackets are the execution confi
guration, which specifies how many threads will execute the kernel. In this
example, you will run 10 GPU threads.*/
