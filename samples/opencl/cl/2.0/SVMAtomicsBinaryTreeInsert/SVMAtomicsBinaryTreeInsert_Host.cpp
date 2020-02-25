/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include "SVMAtomicsBinaryTreeInsert_Host.hpp"

void svm_mutex_init(svm_mutex* lock, int value) {
    atomic_store_explicit(&lock->count, value, std::memory_order_release);
}

void svm_mutex_lock(svm_mutex* lock) {
  int expected = SVM_MUTEX_UNLOCK;
  while(!atomic_compare_exchange_strong_explicit(&lock->count, &expected, SVM_MUTEX_LOCK
					      ,   std::memory_order_seq_cst,std::memory_order_seq_cst)) {
    expected = SVM_MUTEX_UNLOCK;
  }
}

void svm_mutex_unlock(svm_mutex* lock) {
  atomic_store_explicit(&lock->count, SVM_MUTEX_UNLOCK, std::memory_order_release);
}

void initialize_nodes(node *data, size_t num_nodes, int seed)
{
	node *tmp_node;
	long val;

	srand(seed);
	for (size_t i = 0; i < num_nodes; i++)
	{
		tmp_node = &(data[i]);

		val = (((rand() & 255)<<8 | (rand() & 255))<<8 | (rand() & 255))<<7 | (rand() & 127);

		(tmp_node->value) = val;
		tmp_node->left = NULL;
		tmp_node->right = NULL;
		tmp_node->parent = NULL;
		tmp_node->visited = 0;
		tmp_node->childDevType = -1;

		svm_mutex_init(&tmp_node->mutex_node, SVM_MUTEX_UNLOCK);
	}
}

node* cpuMakeBinaryTree(size_t numNodes, node* inroot)
{
  node* root = NULL;
  node* data;
  node* nextData;

  if (NULL != inroot)
  {
      /* allocate first node to root */
      data     = (node *)inroot;
      nextData = data;
      root     = nextData;

      /* iterative tree insert */
      for (size_t i = 1; i < numNodes; ++i)
      {
	  nextData = nextData + 1;

	  insertNode(nextData, root);
      }
  }

  return root;
}

void insertNode(node* tmpData, node* root)
{	
	node* nextNode     = root;
	node* tmp_parent     = NULL;
	node* nextData;

	nextData = tmpData;
	long key = nextData->value;
	long flag = 0;
	int done = 0;


	while (nextNode)
	{
		tmp_parent = nextNode;
		flag = (key - (nextNode->value));
		nextNode = (flag < 0) ? nextNode->left : nextNode->right;
	}
	
	node *child = nextNode;

	do
	{
		svm_mutex *parent_mutex = &tmp_parent->mutex_node;
		svm_mutex_lock(parent_mutex);

		child = (flag < 0) ? tmp_parent->left : tmp_parent->right;

		if(child)
		{
			//Parent node has been updated since last check. Get the new parent and iterate again
			tmp_parent = child;
		}
		else
		{
			//Insert the node
			tmp_parent->left = (flag < 0) ? nextData : tmp_parent->left ;
				
			tmp_parent->right = (flag >= 0) ? nextData : tmp_parent->right ;
	
			//Whether host only insert (childDevType=100) or both host and device  (childDevType=300)
			if (tmp_parent->childDevType == -1 || tmp_parent->childDevType == 100)
			   tmp_parent->childDevType = 100;
			else
			   tmp_parent->childDevType = 300;
			
			nextData->parent = tmp_parent;
			nextData->visited = 1;
			done = 1;
		}

		svm_mutex_unlock(parent_mutex);

	}while (!done);
}
