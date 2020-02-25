/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

/************************* useDefaultBuilt-inPipeFunctions ************************/
/*kernelType=0*/

/**
 *  Producer Kernel writing input data 
 *  into Pipe using pipeWrite function
*/
__kernel 
void pipeWrite(
		__global int *src,
		__write_only pipe int out_pipe)                          
{
	int gid = get_global_id(0);

	reserve_id_t res_id;
	res_id = reserve_write_pipe(out_pipe, 1);

	if(is_valid_reserve_id(res_id))
	{
		if(write_pipe(out_pipe, res_id, 0, &src[gid]) != 0)
		{
			return;
		}
		commit_write_pipe(out_pipe, res_id);
	}
}                                                                         

/**
 *  Consumer Kernel reading input data
 *  from Pipe using pipeRead function
*/
__kernel 
void pipeRead(
		__read_only pipe int in_pipe,
		__global int *dst)                         
{                                                                         
	int gid = get_global_id(0);

	reserve_id_t res_id;
	res_id = reserve_read_pipe(in_pipe, 1);

	if(is_valid_reserve_id(res_id))
	{
		if(read_pipe(in_pipe, res_id, 0, &dst[gid]) != 0)
		{
			return;
		}
		commit_read_pipe(in_pipe, res_id);
	}
}                                                                         



/************************ useWorkGroupBuilt-inPipeFunctions ************************/
/*kernelType=1*/

/**
 *  Producer Kernel writing input data 
 *  into Pipe using pipeWorkGroupWrite function
*/
__kernel 
void pipeWorkGroupWrite(
		__global int *src,
		__write_only pipe int out_pipe)                          
{
	int gid = get_global_id(0);

	__local reserve_id_t res_id;
	res_id = work_group_reserve_write_pipe(out_pipe, get_local_size(0));

	if(is_valid_reserve_id(res_id))
	{
		if(write_pipe(out_pipe, res_id, get_local_id(0), &src[gid]) != 0)
		{
		  return;
		}
		work_group_commit_write_pipe(out_pipe, res_id);
	}
}

/**
 *  Consumer Kernel reading input data
 *  from Pipe using pipeWorkGroupRead function
*/
__kernel 
void pipeWorkGroupRead(
		__read_only pipe int in_pipe,
		__global int *dst)                         
{
    int gid = get_global_id(0);

	__local reserve_id_t res_id;
	res_id = work_group_reserve_read_pipe(in_pipe, get_local_size(0));

	if(is_valid_reserve_id(res_id))
	{
		if(read_pipe(in_pipe, res_id, get_local_id(0), &dst[gid]) != 0)
		{
		  return;
		}
		work_group_commit_read_pipe(in_pipe, res_id);
	}                                                                    
}


/************************ useConvenienceBuilt-inPipeFunctions ************************/
/*kernelType=2*/

/**
 *  Producer Kernel writing input data 
 *  into Pipe using pipeConvenienceWrite function
*/
__kernel 
void pipeConvenienceWrite(
		__global int *src,
		__write_only pipe int out_pipe)                          
{
	int gid = get_global_id(0);

    if(write_pipe(out_pipe, &src[gid]) != 0)
	{
	  return;
	}
}

/**
 *  Consumer Kernel reading input data
 *  from Pipe using pipeConvenienceRead function
*/
__kernel 
void pipeConvenienceRead(
		__read_only pipe int in_pipe,
		__global int *dst)                         
{ 
	int gid = get_global_id(0);
	
	if(read_pipe(in_pipe, &dst[gid]) != 0)
	{
	  return;
	}                                                                        
}




/****************************************** Multiple Pipe Uses ***************************************/
/************************* useDefaultBuilt-inPipeFunctions ************************/
/*kernelType=0*/

/**
 *  Producer Kernel writing input data 
 *  into multple Pipes using multiplePipeWrite function
*/
__kernel 
void multiplePipeWrite(
		__global int *src,
		__write_only pipe int out_pipe1,
		__write_only pipe int out_pipe2,
		__write_only pipe int out_pipe3,
		__write_only pipe int out_pipe4,
		int numPipes)                          
{
	int gid = get_global_id(0);
	reserve_id_t res_id;
	
	if(gid < (get_global_size(0))/numPipes)
	{
		res_id = reserve_write_pipe(out_pipe1, 1);
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe1, res_id, 0, &src[gid]);
			commit_write_pipe(out_pipe1, res_id);
		}
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		res_id = reserve_write_pipe(out_pipe2, 1);
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe2, res_id, 0, &src[gid]);
			commit_write_pipe(out_pipe2, res_id);
		}
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		res_id = reserve_write_pipe(out_pipe3, 1);
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe3, res_id, 0, &src[gid]);
			commit_write_pipe(out_pipe3, res_id);
		}
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		res_id = reserve_write_pipe(out_pipe4, 1);
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe4, res_id, 0, &src[gid]);
			commit_write_pipe(out_pipe4, res_id);
		}
	}
}                                                                         

/**
 *  Consumer Kernel reading input data
 *  from multple Pipes using multiplePipeRead function
*/
__kernel 
void multiplePipeRead(
		__read_only pipe int in_pipe1,
		__read_only pipe int in_pipe2,
		__read_only pipe int in_pipe3,
		__read_only pipe int in_pipe4,
		__global int *dst,
		int numPipes)                         
{                                                                         
	int gid = get_global_id(0);
	reserve_id_t res_id;
	
	if(gid < (get_global_size(0))/numPipes)
	{
		res_id = reserve_read_pipe(in_pipe1, 1);
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe1, res_id, 0, &dst[gid]);
			commit_read_pipe(in_pipe1, res_id);
		}
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		res_id = reserve_read_pipe(in_pipe2, 1);
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe2, res_id, 0, &dst[gid]);
			commit_read_pipe(in_pipe2, res_id);
		}
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		res_id = reserve_read_pipe(in_pipe3, 1);
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe3, res_id, 0, &dst[gid]);
			commit_read_pipe(in_pipe3, res_id);
		}
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		res_id = reserve_read_pipe(in_pipe4, 1);
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe4, res_id, 0, &dst[gid]);
			commit_read_pipe(in_pipe4, res_id);
		}
	}
}                                                                         


/************************ useWorkGroupBuilt-inPipeFunctions ************************/
/*kernelType=1*/

/**
 *  Producer Kernel writing input data 
 *  into multple Pipes using multiplePipeWorkGroupWrite function
*/
__kernel 
void multiplePipeWorkGroupWrite(
		__global int *src,
		__write_only pipe int out_pipe1,
		__write_only pipe int out_pipe2,
		__write_only pipe int out_pipe3,
		__write_only pipe int out_pipe4,
		int numPipes)                          
{
	int gid = get_global_id(0);
	__local reserve_id_t res_id;
	
	if(gid < (get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_write_pipe(out_pipe1, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe1, res_id, get_local_id(0), &src[gid]);
			work_group_commit_write_pipe(out_pipe1, res_id);
		}
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_write_pipe(out_pipe2, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe2, res_id, get_local_id(0), &src[gid]);
			work_group_commit_write_pipe(out_pipe2, res_id);
		}
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_write_pipe(out_pipe3, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe3, res_id, get_local_id(0), &src[gid]);
			work_group_commit_write_pipe(out_pipe3, res_id);
		}
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_write_pipe(out_pipe4, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			write_pipe(out_pipe4, res_id, get_local_id(0), &src[gid]);
			work_group_commit_write_pipe(out_pipe4, res_id);
		}
	}
}                                                                         

/**
 *  Consumer Kernel reading input data
 *  from multple Pipes using multiplePipeWorkGroupRead function
*/
__kernel 
void multiplePipeWorkGroupRead(
		__read_only pipe int in_pipe1,
		__read_only pipe int in_pipe2,
		__read_only pipe int in_pipe3,
		__read_only pipe int in_pipe4,
		__global int *dst,
		int numPipes)                         
{                                                                         
	int gid = get_global_id(0);
	__local reserve_id_t res_id;
	
	if(gid < (get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_read_pipe(in_pipe1, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe1, res_id, get_local_id(0), &dst[gid]);
			work_group_commit_read_pipe(in_pipe1, res_id);
		}
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_read_pipe(in_pipe2, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe2, res_id, get_local_id(0), &dst[gid]);
			work_group_commit_read_pipe(in_pipe2, res_id);
		}
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_read_pipe(in_pipe3, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe3, res_id, get_local_id(0), &dst[gid]);
			work_group_commit_read_pipe(in_pipe3, res_id);
		}
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		res_id = work_group_reserve_read_pipe(in_pipe4, get_local_size(0));
		if(is_valid_reserve_id(res_id))
		{
			read_pipe(in_pipe4, res_id, get_local_id(0), &dst[gid]);
			work_group_commit_read_pipe(in_pipe4, res_id);
		}
	}
}


/************************ useConvenienceBuilt-inPipeFunctions ************************/
/*kernelType=2*/

/**
 *  Producer Kernel writing input data 
 *  into multple Pipes using multiplePipeConvenienceWrite function
*/
__kernel 
void multiplePipeConvenienceWrite(
		__global int *src,
		__write_only pipe int out_pipe1,
		__write_only pipe int out_pipe2,
		__write_only pipe int out_pipe3,
		__write_only pipe int out_pipe4,
		int numPipes)                          
{
	int gid = get_global_id(0);
	
	if(gid < (get_global_size(0))/numPipes)
	{
		write_pipe(out_pipe1, &src[gid]);
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		write_pipe(out_pipe2, &src[gid]);
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		write_pipe(out_pipe3, &src[gid]);
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		write_pipe(out_pipe4, &src[gid]);
	}
}                                                                         

/**
 *  Consumer Kernel reading input data
 *  from multple Pipes using multiplePipeConvenienceRead function
*/
__kernel 
void multiplePipeConvenienceRead(
		__read_only pipe int in_pipe1,
		__read_only pipe int in_pipe2,
		__read_only pipe int in_pipe3,
		__read_only pipe int in_pipe4,
		__global int *dst,
		int numPipes)                         
{                                                                         
	int gid = get_global_id(0);
	
	if(gid < (get_global_size(0))/numPipes)
	{
		read_pipe(in_pipe1, &dst[gid]);
	}
	else if(gid < (2*get_global_size(0))/numPipes)
	{
		read_pipe(in_pipe2, &dst[gid]);
	}
	else if(gid < (3*get_global_size(0))/numPipes)
	{
		read_pipe(in_pipe3, &dst[gid]);
	}
	else if(gid < (4*get_global_size(0))/numPipes)
	{
		read_pipe(in_pipe4, &dst[gid]);
	}
}