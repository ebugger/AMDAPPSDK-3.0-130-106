#ifndef __SVM_BINARY_NODE__
#define __SVM_BINARY_NODE__

#define SVM_MUTEX_LOCK    1
#define SVM_MUTEX_UNLOCK  0

#ifndef SVM_DATA_STRUCT_OPENCL_DEVICE

#include <atomic>
#define __global 

#endif

typedef struct {
#ifndef SVM_DATA_STRUCT_OPENCL_DEVICE
	std::atomic<int> count;
#else
	volatile int count;
#endif
} svm_mutex;

typedef struct bin_tree
{
	long value;				// Value at a node
	__global struct bin_tree *left;      	// Pointer to the left node
	__global struct bin_tree *right;     	// Pointer to the right node
	__global struct bin_tree *parent;     	// Pointer to the parent node
	svm_mutex mutex_node;
        int	childDevType;			// Indicates which device inserted its child nodes
						// 100 - denotes contains child nodes inserted by host only
						// 200 - denotes contains child nodes inserted by device only
						// 300 - denotes contains child nodes inserted by both host and device

	int     visited;			// Indicates whether the node is inserted to binary tree
	
} node;


#endif //__SVM_BINARY_NODE__

