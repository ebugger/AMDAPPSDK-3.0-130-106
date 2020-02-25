#ifndef _REGION_GROWING_LL_
#define _REGION_GROWING_LL_

#include "CLUtil.hpp"
#include "RegionGrowingConst.hpp"

/*
 * RGSLinkedList:
 * linked list class to enqueue and dequeue pixels.
 */

class RGSLinkedList
{
public:  
  RGSLLNodeType*       pHead;
  RGSLLNodeType*       pMid;
  RGSLLNodeType*       pTail;
  cl_uint              length;

  /* constructuor-destructor pair */
  RGSLinkedList();
  ~RGSLinkedList();

  /* push and pop */
  void           reset();
  void           push(RGSLLNodeType* pNode);
  RGSLLNodeType* pop();
};

RGSLinkedList::RGSLinkedList()
{
  pHead              = new RGSLLNodeType;
  pHead->index.x     = -1; 
  pHead->index.y     = -1; 
  pHead->pNext       = NULL;


  pMid              = new RGSLLNodeType;
  pMid->index.x     = -1; 
  pMid->index.y     = -1; 
  pMid->pNext       = NULL;

  pHead->pNext      = pMid;

  pTail             = pMid;

  length            = 0;
}

RGSLinkedList::~RGSLinkedList()
{
  RGSLLNodeType*       pTmp;
  while(pHead)
    {
      pTmp  = pHead->pNext;
      delete pHead;
      pHead = pTmp;
    }
}

void RGSLinkedList::reset()
{
  
  RGSLLNodeType*       pTmp;

  pTail->pNext = pHead;
  pHead->pNext = NULL;

  pTmp   = pHead;
  pHead  = pMid;
  pMid   = pTmp;

  pTail  = pMid;

  length = 0;
}

void RGSLinkedList::push(RGSLLNodeType* pNode)
{
  
  pTail->pNext = pNode;
  pNode->pNext = NULL;
  pTail        = pTail->pNext;

  length      += 1;
}

RGSLLNodeType* RGSLinkedList::pop()
{
  RGSLLNodeType* pTmp;

  pTmp = pHead->pNext;
  
  if(pTmp != pMid)
    {
      pHead->pNext = pTmp->pNext;
    }
  else
    {
      pTmp = NULL;
    }

  return pTmp;
}

#endif
