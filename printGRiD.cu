/***
nvcc -std=c++11 -o printGRiD.exe printGRiD.cu -gencode arch=compute_86,code=sm_86 -O3 -ftz=true -prec-div=false -prec-sqrt=false
***/

#include <random>
#include <algorithm>
#include "grid.cuh"
#define RANDOM_MEAN 0
#define RANDOM_STDEV 1
std::default_random_engine randEng(1337); // fixed seed
std::normal_distribution<double> randDist(RANDOM_MEAN, RANDOM_STDEV); //mean followed by stdiv
template <typename T>
T getRand(){return static_cast<T>(randDist(randEng));}

template <typename T>
__host__
void test(){
	T gravity = static_cast<T>(9.81);
	dim3 dimms(grid::SUGGESTED_THREADS,1,1);
	cudaStream_t *streams = grid::init_grid<T>();
	grid::robotModel<T> *d_robotModel = grid::init_robotModel<T>();
	grid::gridData<T> *hd_data = grid::init_gridData<T,1>();
	
	// load q,qd,u
	for(int j = 0; j < grid::NUM_JOINTS; j++){
		hd_data->h_q_qd_u[j] = getRand<double>(); 
		hd_data->h_q_qd_u[j+grid::NUM_JOINTS] = getRand<double>(); 
		hd_data->h_q_qd_u[j+2*grid::NUM_JOINTS] = getRand<double>();
	}
	gpuErrchk(cudaMemcpy(hd_data->d_q_qd_u,hd_data->h_q_qd_u,3*grid::NUM_JOINTS*sizeof(T),cudaMemcpyHostToDevice));
	gpuErrchk(cudaDeviceSynchronize());

	printf("q,qd,u\n");
	printMat<T,1,grid::NUM_JOINTS>(hd_data->h_q_qd_u,1);
	printMat<T,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[grid::NUM_JOINTS],1);
	printMat<T,1,grid::NUM_JOINTS>(&hd_data->h_q_qd_u[2*grid::NUM_JOINTS],1);

	printf("c\n");
	grid::inverse_dynamics<T,false,false>(hd_data,d_robotModel,gravity,1,dim3(1,1,1),dimms,streams);
	printMat<T,1,grid::NUM_JOINTS>(hd_data->h_c,1);

	printf("Minv\n");
	grid::direct_minv<T,false>(hd_data,d_robotModel,1,dim3(1,1,1),dimms,streams);
	printMat<T,grid::NUM_JOINTS,grid::NUM_JOINTS>(hd_data->h_Minv,grid::NUM_JOINTS);

	printf("qdd\n");
	grid::forward_dynamics<T>(hd_data,d_robotModel,gravity,1,dim3(1,1,1),dimms,streams);
	printMat<T,1,grid::NUM_JOINTS>(hd_data->h_qdd,1);

	grid::inverse_dynamics_gradient<T,true,false>(hd_data,d_robotModel,gravity,1,dim3(1,1,1),dimms,streams);
	printf("dc_dq\n");
	printMat<T,grid::NUM_JOINTS,grid::NUM_JOINTS>(hd_data->h_dc_du,grid::NUM_JOINTS);
	printf("dc_dqd\n");
	printMat<T,grid::NUM_JOINTS,grid::NUM_JOINTS>(&hd_data->h_dc_du[grid::NUM_JOINTS*grid::NUM_JOINTS],grid::NUM_JOINTS);

	grid::forward_dynamics_gradient<T,false>(hd_data,d_robotModel,gravity,1,dim3(1,1,1),dimms,streams);
	printf("df_dq\n");
	printMat<T,grid::NUM_JOINTS,grid::NUM_JOINTS>(hd_data->h_df_du,grid::NUM_JOINTS);
	printf("df_dqd\n");
	printMat<T,grid::NUM_JOINTS,grid::NUM_JOINTS>(&hd_data->h_df_du[grid::NUM_JOINTS*grid::NUM_JOINTS],grid::NUM_JOINTS);

	grid::close_grid<T>(streams,d_robotModel,hd_data);
}

int main(void){
	test<float>(); return 0;
}