// #include "grid.cuh"
#include "/home/a2rlab4/GRiDBenchmarks/GRiD/grid.cuh"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cuda_runtime.h>


int iiwa(){
    grid::gridData<float> *hd_data = grid::init_gridData<float,1>();
    grid::robotModel<float> *d_robotModel = grid::init_robotModel<float>();;
    const int num_timesteps = 1;
    float gravity = static_cast<float>(9.81);
    dim3 dimms(grid::SUGGESTED_THREADS,1,1);
    cudaStream_t *streams = grid::init_grid<float>();
    hd_data->h_q_qd_u[0] = 1.24;
    hd_data->h_q_qd_u[1] = 0.13;
    hd_data->h_q_qd_u[2] = -0.17;
    hd_data->h_q_qd_u[3] = 1.33;
    hd_data->h_q_qd_u[4] = 0.22;
    hd_data->h_q_qd_u[5] = -0.56;
    hd_data->h_q_qd_u[6] = 0.99;

    hd_data->h_q_qd_u[7] = 0;
    hd_data->h_q_qd_u[8] = 0;
    hd_data->h_q_qd_u[9] = 0;
    hd_data->h_q_qd_u[10] = 0;
    hd_data->h_q_qd_u[11] = 0;
    hd_data->h_q_qd_u[12] = 0;
    hd_data->h_q_qd_u[13] = 0;

    hd_data->h_q_qd_u[14] = 0;
    hd_data->h_q_qd_u[15] = 0;
    hd_data->h_q_qd_u[16] = 0;
    hd_data->h_q_qd_u[17] = 0;
    hd_data->h_q_qd_u[18] = 0;
    hd_data->h_q_qd_u[19] = 0;
    hd_data->h_q_qd_u[20] = 0;

    gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(float),cudaMemcpyHostToDevice));
    gpuErrchk(cudaDeviceSynchronize());

    printf("q,qd,u\n");
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

    printf("aba\n");
    grid::aba<float>(hd_data, d_robotModel, gravity, 1, dim3(1,1,1), dimms, streams);
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);
    return 0;
}

int hyq(){
    grid::gridData<float> *hd_data = grid::init_gridData<float,1>();
    grid::robotModel<float> *d_robotModel = grid::init_robotModel<float>();;
    const int num_timesteps = 1;
    float gravity = static_cast<float>(9.81);
    dim3 dimms(grid::SUGGESTED_THREADS,1,1);
    cudaStream_t *streams = grid::init_grid<float>();
    hd_data->h_q_qd_u[0] = 0.8;
    hd_data->h_q_qd_u[1] = 0.3;
    hd_data->h_q_qd_u[2] = 1;
    hd_data->h_q_qd_u[3] = 0.2;
    hd_data->h_q_qd_u[4] = 0.7;
    hd_data->h_q_qd_u[5] = 0.6;
    hd_data->h_q_qd_u[6] = 0.4;
    hd_data->h_q_qd_u[7] = 0.5;
    hd_data->h_q_qd_u[8] = 0.3;
    hd_data->h_q_qd_u[9] = 0.8;
    hd_data->h_q_qd_u[10] = 0.1;
    hd_data->h_q_qd_u[11] = 0.2;

    hd_data->h_q_qd_u[12] = 0.1;
    hd_data->h_q_qd_u[13] = 0.1;
    hd_data->h_q_qd_u[14] = 0.1;
    hd_data->h_q_qd_u[15] = 0.1;
    hd_data->h_q_qd_u[16] = 0.1;
    hd_data->h_q_qd_u[17] = 0.1;
    hd_data->h_q_qd_u[18] = 0.1;
    hd_data->h_q_qd_u[19] = 0.1;
    hd_data->h_q_qd_u[20] = 0.1;
    hd_data->h_q_qd_u[21] = 0.1;
    hd_data->h_q_qd_u[22] = 0.1;
    
    hd_data->h_q_qd_u[23] = 0.1;
    hd_data->h_q_qd_u[24] = 0.1;
    hd_data->h_q_qd_u[25] = 0.1;
    hd_data->h_q_qd_u[26] = 0.1;
    hd_data->h_q_qd_u[27] = 0.1;
    hd_data->h_q_qd_u[28] = 0.1;
    hd_data->h_q_qd_u[29] = 0.1;
    hd_data->h_q_qd_u[30] = 0.1;
    hd_data->h_q_qd_u[31] = 0.1;
    hd_data->h_q_qd_u[32] = 0.1;


    gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(float),cudaMemcpyHostToDevice));
    gpuErrchk(cudaDeviceSynchronize());

    printf("q,qd,u\n");
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

    printf("aba\n");
    grid::aba<float>(hd_data, d_robotModel, gravity, 1, dim3(1,1,1), dimms, streams);
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);
    
    return 0;
}

int hyq_missing_limb(){
    grid::gridData<float> *hd_data = grid::init_gridData<float,1>();
    grid::robotModel<float> *d_robotModel = grid::init_robotModel<float>();;
    const int num_timesteps = 1;
    float gravity = static_cast<float>(9.81);
    dim3 dimms(grid::SUGGESTED_THREADS,1,1);
    cudaStream_t *streams = grid::init_grid<float>();
    hd_data->h_q_qd_u[0] = 0.8;
    hd_data->h_q_qd_u[1] = 0.3;
    hd_data->h_q_qd_u[2] = 1;
    hd_data->h_q_qd_u[3] = 0.2;
    hd_data->h_q_qd_u[4] = 0.7;
    hd_data->h_q_qd_u[5] = 0.6;
    hd_data->h_q_qd_u[6] = 0.4;
    hd_data->h_q_qd_u[7] = 0.4;
    hd_data->h_q_qd_u[8] = 0.3;
    hd_data->h_q_qd_u[9] = 0.8;
    hd_data->h_q_qd_u[10] = 0.1;
    
    hd_data->h_q_qd_u[11] = 0;
    hd_data->h_q_qd_u[12] = 0;
    hd_data->h_q_qd_u[13] = 0;
    hd_data->h_q_qd_u[14] = 0;
    hd_data->h_q_qd_u[15] = 0;
    hd_data->h_q_qd_u[16] = 0;
    hd_data->h_q_qd_u[17] = 0;
    hd_data->h_q_qd_u[18] = 0;
    hd_data->h_q_qd_u[19] = 0;
    hd_data->h_q_qd_u[20] = 0;
    hd_data->h_q_qd_u[21] = 0;
    
    hd_data->h_q_qd_u[22] = 0;
    hd_data->h_q_qd_u[23] = 0;
    hd_data->h_q_qd_u[24] = 0;
    hd_data->h_q_qd_u[25] = 0;
    hd_data->h_q_qd_u[26] = 0;
    hd_data->h_q_qd_u[27] = 0;
    hd_data->h_q_qd_u[28] = 0;
    hd_data->h_q_qd_u[29] = 0;
    hd_data->h_q_qd_u[30] = 0;
    hd_data->h_q_qd_u[31] = 0;
    hd_data->h_q_qd_u[32] = 0;


    gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(float),cudaMemcpyHostToDevice));
    gpuErrchk(cudaDeviceSynchronize());

    printf("q,qd,u\n");
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

    printf("aba\n");
    grid::aba<float>(hd_data, d_robotModel, gravity, 1, dim3(1,1,1), dimms, streams);
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);
    
    return 0;
}

int atlas(){
    grid::gridData<float> *hd_data = grid::init_gridData<float,1>();
    grid::robotModel<float> *d_robotModel = grid::init_robotModel<float>();;
    const int num_timesteps = 1;
    float gravity = static_cast<float>(9.81);
    dim3 dimms(grid::SUGGESTED_THREADS,1,1);
    cudaStream_t *streams = grid::init_grid<float>();
    hd_data->h_q_qd_u[0] = 0.8;
    hd_data->h_q_qd_u[1] = 0.3;
    hd_data->h_q_qd_u[2] = 1;
    hd_data->h_q_qd_u[3] = 0.2;
    hd_data->h_q_qd_u[4] = 0.7;
    hd_data->h_q_qd_u[5] = 0.6;
    hd_data->h_q_qd_u[6] = 0.4;
    hd_data->h_q_qd_u[7] = 0.4;
    hd_data->h_q_qd_u[8] = 0.3;
    hd_data->h_q_qd_u[9] = 0.8;
    hd_data->h_q_qd_u[10] = 0.1;
    hd_data->h_q_qd_u[11] = 0.2;
    hd_data->h_q_qd_u[12] = 0.3;
    hd_data->h_q_qd_u[13] = 0.5;
    hd_data->h_q_qd_u[14] = 0.4;
    hd_data->h_q_qd_u[15] = 0.37;
    hd_data->h_q_qd_u[16] = 0.9;
    hd_data->h_q_qd_u[17] = 0.53;
    hd_data->h_q_qd_u[18] = 0.2;
    hd_data->h_q_qd_u[19] = 1;
    hd_data->h_q_qd_u[20] = 0.6;
    hd_data->h_q_qd_u[21] = 0.8;
    hd_data->h_q_qd_u[22] = 0.9;
    hd_data->h_q_qd_u[23] = 0.3;
    hd_data->h_q_qd_u[24] = 0.7;
    hd_data->h_q_qd_u[25] = 0.44;
    hd_data->h_q_qd_u[26] = 0.28;
    hd_data->h_q_qd_u[27] = 0.6;
    hd_data->h_q_qd_u[28] = 1;
    hd_data->h_q_qd_u[29] = 0.1;

    hd_data->h_q_qd_u[60] = 0.1;
    hd_data->h_q_qd_u[61] = 0.1;
    hd_data->h_q_qd_u[62] = 0.1;
    hd_data->h_q_qd_u[63] = 0.1;
    hd_data->h_q_qd_u[64] = 0.1;
    hd_data->h_q_qd_u[65] = 0.1;
    hd_data->h_q_qd_u[66] = 0.1;
    hd_data->h_q_qd_u[67] = 0.1;
    hd_data->h_q_qd_u[68] = 0.1;
    hd_data->h_q_qd_u[69] = 0.1;
    hd_data->h_q_qd_u[70] = 0.1;
    hd_data->h_q_qd_u[71] = 0.1;
    hd_data->h_q_qd_u[72] = 0.1;
    hd_data->h_q_qd_u[73] = 0.1;
    hd_data->h_q_qd_u[74] = 0.1;
    hd_data->h_q_qd_u[75] = 0.1;
    hd_data->h_q_qd_u[76] = 0.1;
    hd_data->h_q_qd_u[77] = 0.1;
    hd_data->h_q_qd_u[78] = 0.1;
    hd_data->h_q_qd_u[79] = 0.1;
    hd_data->h_q_qd_u[80] = 0.1;
    hd_data->h_q_qd_u[81] = 0.1;
    hd_data->h_q_qd_u[82] = 0.1;
    hd_data->h_q_qd_u[83] = 0.1;
    hd_data->h_q_qd_u[84] = 0.1;
    hd_data->h_q_qd_u[85] = 0.1;
    hd_data->h_q_qd_u[86] = 0.1;
    hd_data->h_q_qd_u[87] = 0.1;
    hd_data->h_q_qd_u[88] = 0.1;
    hd_data->h_q_qd_u[89] = 0.1;

    hd_data->h_q_qd_u[30] = 0.1;
    hd_data->h_q_qd_u[31] = 0.1;
    hd_data->h_q_qd_u[32] = 0.1;
    hd_data->h_q_qd_u[33] = 0.1;
    hd_data->h_q_qd_u[34] = 0.1;
    hd_data->h_q_qd_u[35] = 0.1;
    hd_data->h_q_qd_u[36] = 0.1;
    hd_data->h_q_qd_u[37] = 0.1;
    hd_data->h_q_qd_u[38] = 0.1;
    hd_data->h_q_qd_u[39] = 0.1;
    hd_data->h_q_qd_u[40] = 0.1;
    hd_data->h_q_qd_u[41] = 0.1;
    hd_data->h_q_qd_u[42] = 0.1;
    hd_data->h_q_qd_u[43] = 0.1;
    hd_data->h_q_qd_u[44] = 0.1;
    hd_data->h_q_qd_u[45] = 0.1;
    hd_data->h_q_qd_u[46] = 0.1;
    hd_data->h_q_qd_u[47] = 0.1;
    hd_data->h_q_qd_u[48] = 0.1;
    hd_data->h_q_qd_u[49] = 0.1;
    hd_data->h_q_qd_u[50] = 0.1;
    hd_data->h_q_qd_u[51] = 0.1;
    hd_data->h_q_qd_u[52] = 0.1;
    hd_data->h_q_qd_u[53] = 0.1;
    hd_data->h_q_qd_u[54] = 0.1;
    hd_data->h_q_qd_u[55] = 0.1;
    hd_data->h_q_qd_u[56] = 0.1;
    hd_data->h_q_qd_u[57] = 0.1;
    hd_data->h_q_qd_u[58] = 0.1;
    hd_data->h_q_qd_u[59] = 0.1;



    gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(float),cudaMemcpyHostToDevice));
    gpuErrchk(cudaDeviceSynchronize());

    printf("q,qd,u\n");
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
    printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

    printf("aba\n");
    grid::aba<float>(hd_data, d_robotModel, gravity, 1, dim3(1,1,1), dimms, streams);
    printMat<float,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);
    
    return 0;
}

int main() {    
    //uncomment the line for the URDF you are using

    iiwa();
    //hyq();
    //hyq_missing_limb();
    // atlas();
    return 0;
}

// int main() {
//     grid::gridData<float> *hd_data = grid::init_gridData<float,1>();
//     grid::robotModel<float> *d_robotModel = grid::init_robotModel<float>();;
//     const int num_timesteps = 1;
//     float gravity = static_cast<float>(9.81);
//     dim3 dimms(grid::SUGGESTED_THREADS,1,1);
//     cudaStream_t *streams = grid::init_grid<float>();
//     hd_data->h_q_qd_u[0] = 0.8;
//     hd_data->h_q_qd_u[1] = 0.3;
//     hd_data->h_q_qd_u[2] = 1;
//     hd_data->h_q_qd_u[3] = 0.2;
//     hd_data->h_q_qd_u[4] = 0.7;
//     hd_data->h_q_qd_u[5] = 0.6;
//     hd_data->h_q_qd_u[6] = 0.4;

//     hd_data->h_q_qd_u[7] = 0;
//     hd_data->h_q_qd_u[8] = 0;
//     hd_data->h_q_qd_u[9] = 0;
//     hd_data->h_q_qd_u[10] = 0;
//     hd_data->h_q_qd_u[11] = 0;
//     hd_data->h_q_qd_u[12] = 0;
//     hd_data->h_q_qd_u[13] = 0;

//     hd_data->h_q_qd_u[14] = 0;
//     hd_data->h_q_qd_u[15] = 0;
//     hd_data->h_q_qd_u[16] = 0;
//     hd_data->h_q_qd_u[17] = 0;
//     hd_data->h_q_qd_u[18] = 0;
//     hd_data->h_q_qd_u[19] = 0;
//     hd_data->h_q_qd_u[20] = 0;

//     gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(float),cudaMemcpyHostToDevice));
//     gpuErrchk(cudaDeviceSynchronize());

//     printf("q,qd,u\n");
//     printMat<float,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
//     printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
//     printMat<float,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

//     printf("aba\n");
//     grid::aba<float>(hd_data, d_robotModel, gravity, 1, dim3(1,1,1), dimms, streams);
//     printMat<float,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);
//     return 0;
// }
