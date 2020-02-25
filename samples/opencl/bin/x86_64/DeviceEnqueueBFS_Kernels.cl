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


/******************** Device-Enqueue BFS Kernel ********************/


#define INIFINITY 100000000
#define getNumChild(d_rowPtr, node)  (d_rowPtr[node+1] - d_rowPtr[node])
#define getChildNode(d_colIndex, offset)  d_colIndex[offset]

__global atomic_uint g_totalNeighborsCount = ATOMIC_VAR_INIT(0);


/*
*  BFS Kernel Declaration
*/
__kernel 
void deviceEnqueueBFSKernel(__global uint *d_rowPtr, 
							__global uint *d_colIndex,
    						__global uint *d_dist,
							__read_only pipe uint d_vertexFrontier_inPipe,
							__write_only pipe uint d_edgeFrontier_outPipe,
							uint parentNodeLevel );



/**
 *  @brief    This kenrel is called for initializing vertex queue, 
			  (a write-pipe memory object). For very first time, 
			  the write-pipe object is initialized with root vertex. 

			  It is called only from host and only once with one 
			  work-item. 
			  
 *  @param    out_pipe writing root vertex into write-pipe
 */

__kernel 
void pipeWrite(__write_only pipe int out_pipe
		      )                          
{
	int root = 0; 

	reserve_id_t res_id;
	res_id = reserve_write_pipe(out_pipe, 1);

	if(is_valid_reserve_id(res_id))
	{
		if(write_pipe(out_pipe, res_id, 0, &root) != 0)
		{
			return;
		}
		commit_write_pipe(out_pipe, res_id);
	}
}


/*
*  Dummy Kernel
*  @brief     This kernel is launched as a global
			  synchronization event between two levels.
			  This synchronization event make sure that 
			  child kernel should have exact value of 
			  global_work_item size.
*/
__kernel 
void deviceEnqueueDummyKernel(__global uint *d_rowPtr, 
							  __global uint *d_colIndex,
    						  __global uint *d_dist,
							  __read_only pipe uint d_edgeFrontier_outPipe,
							  __write_only pipe uint d_vertexFrontier_inPipe,
							  uint parentNodeLevel )
{
	uint globalThreads = atomic_load_explicit(&g_totalNeighborsCount, memory_order_seq_cst, memory_scope_device);

	if(globalThreads == 0)   // don't need to launch kernel if there is no child
			return;

	queue_t q = get_default_queue();
	ndrange_t ndrange1 = ndrange_1D(globalThreads);
		
	void (^bfs_device_enqueue_wrapper_blk)(void) = ^{deviceEnqueueBFSKernel(d_rowPtr,
																			d_colIndex,
																			d_dist, 
																			d_edgeFrontier_outPipe,
																			d_vertexFrontier_inPipe,
																			parentNodeLevel );};  
	int err_ret = enqueue_kernel(q, CLK_ENQUEUE_FLAGS_WAIT_KERNEL, ndrange1, bfs_device_enqueue_wrapper_blk);

   	if(err_ret != 0)
	{
		return;
    }
}


/*
*  Parallelly visit all immediate neighbour nodes 
*/

/**
 *  @brief    Traverse the graph using BFS. Graph data is stored in CSR 
			  (Compressed Row Sparse) format. It implements level-synchronous
			  algorithm to Parallelly visit all immediate neighbour nodes. It 
			  uses 2 pipes, one for reading the vertices of current level and
			  another one for writing all neighbour vertices of currently visited
			  nodes.

			  It starts with root node. Host launches the kernel only once from
			  very beginning with one work-item, to visit the root node. 

			  At each iteration, kernel 
			  a. first reads vertices of the current level from read-pipe and 
			  b. then expands all neighbour vertices into a write pipe and finally 
			  c. it launches the same kernel by swapping write pipe into read pipe.
			  Kernel repeats the same steps a and b, till it visits all the nodes.
			  
			  It also computes distance of each node from root node.
			  
 *  @param    d_rowPtr input buffer stores the location in colIndex array where each row starts
 *  @param    d_colIndex input buffer stores column indexes of each non-zero element
 *  @param    d_dist output buffer stores distance of each node from root node
 *  @param    d_vertexFrontier_inPipe a read pipe or an vertex frontier queue stores list of nodes to be visited in current level
 *  @param    d_edgeFrontier_outPipe a write pipe or an edge frontier queue stores list of the neighbour nodes of current visited nodes
 *  @param    parentNodeLevel the parent node level
*/

__kernel 
void deviceEnqueueBFSKernel(__global uint *d_rowPtr, 
							__global uint *d_colIndex,
    						__global uint *d_dist,
							__read_only pipe uint d_vertexFrontier_inPipe,
							__write_only pipe uint d_edgeFrontier_outPipe,
							uint parentNodeLevel )
{ 
	uint gid = get_global_id(0);
	uint lid = get_local_id(0);
	reserve_id_t res_read_id, res_write_id;

	uint node, childNode;
	uint offset = 0, numChildPerNode = 0, globalThreads = 0, currentLevel = 0, tmpNeighborsCount = 0, tmpTotalNeighborsCount = 0, wgCnt = 0;
	
	atomic_store_explicit(&g_totalNeighborsCount,0,memory_order_seq_cst, memory_scope_device);
	
	// read current level's vertices to be visited (/* reading from pipe */)
	res_read_id = reserve_read_pipe(d_vertexFrontier_inPipe, 1);
	if(is_valid_reserve_id(res_read_id))
	{
		if(read_pipe(d_vertexFrontier_inPipe, res_read_id, 0, &node) != 0)
		{
			return;
		}
		commit_read_pipe(d_vertexFrontier_inPipe, res_read_id);
	}
		
	// check whether node is visited
	if(d_dist[node] == INIFINITY)  
	{
		// if not visited, visit the node
		if(node == 0)
			d_dist[node] = 0;
		else
			d_dist[node] = parentNodeLevel + 1; 

		// calculate all neighbour nodes for next level
		offset = d_rowPtr[node];
		numChildPerNode = getNumChild(d_rowPtr, node) ;
		
		// expand these neighbours for the next level, only when it has not been visited  (/* Writing into Pipe */)
		for(int i = 0; i < numChildPerNode; i++)
		{
			childNode = getChildNode(d_colIndex, offset+i);
			if(d_dist[childNode] == INIFINITY) 
			{
				res_write_id = reserve_write_pipe(d_edgeFrontier_outPipe, 1);

				if(is_valid_reserve_id(res_write_id))
				{
					if(write_pipe(d_edgeFrontier_outPipe, res_write_id, 0, &childNode) != 0)
					{
						return;
					}
					commit_write_pipe(d_edgeFrontier_outPipe, res_write_id);
				}

				tmpNeighborsCount++;
			}
		}

		//summing number of Neighbours within work group
		wgCnt = work_group_reduce_add(tmpNeighborsCount);
	}

	//summing total number of Neighbours across all work-groups
	if(lid == 0)
	{
		atomic_fetch_add_explicit(&g_totalNeighborsCount, wgCnt, memory_order_seq_cst, memory_scope_device);  /* Counting launch size of next level */
	}
	

	//let us relaunch the kernel for next level
	if(gid == 0)
	{
		globalThreads = 1;
		currentLevel = d_dist[node];

		// lets relaunch the kernel for its child node
		queue_t q = get_default_queue();
		ndrange_t ndrange1 = ndrange_1D(globalThreads);
		
		void (^bfsDummy_device_enqueue_wrapper_blk)(void) = ^{deviceEnqueueDummyKernel(d_rowPtr,
																					   d_colIndex,
																				   	   d_dist, 
																				       d_edgeFrontier_outPipe,
																					   d_vertexFrontier_inPipe,
																					   currentLevel );};  
		int err_ret = enqueue_kernel(q, CLK_ENQUEUE_FLAGS_WAIT_KERNEL, ndrange1, bfsDummy_device_enqueue_wrapper_blk);

    	if(err_ret != 0)
		{
			return;
	    }
	}
}