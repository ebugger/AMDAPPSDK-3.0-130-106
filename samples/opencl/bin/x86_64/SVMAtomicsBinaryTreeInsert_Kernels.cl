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

#define SVM_DATA_STRUCT_OPENCL_DEVICE

#include "SVMBinaryNode.h"

/*
 * This kernel inserts a node on an BST.
 * Arguments:	rootNode	- The root node the binary tree
 *		devStartNode	- The first node which device has to begin inserting
 *		gpu_nodes	- Number of nodes device has to insert
 */

__kernel void binTreeInsert(
			__global void *rootNode,
			__global void *devStartNode,
			const size_t gpu_nodes
			)
{
	__global volatile svm_mutex *tmp_mutex;
	__global node *tmp_node, *tmp_parent, *new_node; 
    
	__global node *root = (__global node *)rootNode;
	__global node *data = (__global node *)devStartNode;
	long flag;
	long key;
	
	size_t gidx = get_global_id(0);
	
	//return if beyond limits
	if (gidx >= gpu_nodes)
	{
		return;
	}

	flag = 0;
	tmp_node = root;

	tmp_parent = root;
	new_node = &(data[gidx]);

	key = (new_node->value);

	/* Search the parent node. 
	 * Multiple work-items in the a work-group run this part. */	
	while (tmp_node) 
	{
		tmp_parent = tmp_node;
		flag = (key - (tmp_node->value));
		tmp_node = (flag < 0) ? tmp_node->left : tmp_node->right;
	} 
	
	__global node *child = tmp_node;
	int done = 0;
	tmp_mutex = &tmp_parent->mutex_node;
        int exFlag, expected;

	do
	{
		tmp_mutex = &tmp_parent->mutex_node;
		expected = SVM_MUTEX_UNLOCK;
		
		exFlag = atomic_compare_exchange_strong_explicit((atomic_int *)&tmp_mutex->count, &expected, SVM_MUTEX_LOCK, memory_order_seq_cst,memory_order_seq_cst, memory_scope_all_svm_devices);


		//If Parent node lock is successful
		if(exFlag)
		{
			child = (flag < 0) ? tmp_parent->left : tmp_parent->right;
			if(child)
			{
				//Parent node has been updated since last check. Get the new parent and iterate again
				tmp_parent = child;
			}
			else
			{
				//Insert the node			
				tmp_parent->left = (flag < 0) ? new_node : tmp_parent->left ;
				
				tmp_parent->right = (flag >= 0) ? new_node : tmp_parent->right ;

				//Whether device only insert (childDevType=200) or both host and device  (childDevType=300)
				if (tmp_parent->childDevType == -1 || tmp_parent->childDevType == 200)
				    tmp_parent->childDevType = 200;
				else
				    tmp_parent->childDevType = 300;
				new_node->parent = tmp_parent;

				new_node->visited = 1;

				done = 1;
			}

			expected = SVM_MUTEX_LOCK;
			
			atomic_compare_exchange_strong_explicit((atomic_int *)&tmp_mutex->count, &expected, SVM_MUTEX_UNLOCK, memory_order_seq_cst,memory_order_seq_cst, memory_scope_all_svm_devices);
		}
		
		atomic_work_item_fence(CLK_GLOBAL_MEM_FENCE, memory_order_seq_cst, memory_scope_all_svm_devices);
		
	}while (!done);
}
