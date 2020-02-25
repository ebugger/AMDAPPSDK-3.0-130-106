#ifndef _REGION_GROWING_CPU_
#define _REGION_GROWING_CPU_

#include "CLUtil.hpp"
#include "RegionGrowingConst.hpp"
#include "RegionGrowingLL.hpp"

/*
 * RGS:
 * This class implements the verification code for seeded region growing segmentation.
 * A linked list class RGSLinkedList is used to enqueue and dequeue the pixels.
 */
class RGS
{
public:
  cl_int           numRegions;
  RGSRegionType*   pRegionInfo;
  RGSLinkedList*   pList;
  cl_uchar4*       pImage;
  cl_uint          width;
  cl_uint          height;

  cl_int2          pNeigh[RGS_NEIGH];

  /* constructor and destructor pair */
  RGS();
  ~RGS();

  /*
   * init:
   * read input image, seed pixels and various other parameters.
   */
  int     init(cl_uchar4**     inputImageData,
               cl_int2*        pQueuedPixels,
               RGSParamType*   pParams); 

  /*
   * growRegion:
   * top level region growing finction.
   *
   */
  int     growRegion();

  /*
   * queueAllNeighs:
   * for a given pixel pNode, enqueues all its unclassified neighbors for
   * classification.
   *
   */
  int     queueAllNeighs(RGSLLNodeType* pNode);

  /*
   * queueNeigh:
   * enqueues i-th neighbor of pixel pNode.
   */
  int     queueNeigh(RGSLLNodeType* pNode, cl_int i);

  /*
   * classify:
   * classifies all enqueued pixels into regions.
   */
  int     classify();

  /*
   * minRegion:
   * finds the region closest to a pixel.
   */
  int     minRegion(cl_float pixLuma);

  /*
   * finalClassify:
   * final classification based on the neighborhood classification of a pixel.
   */
  int     finalClassify();
};

RGS::RGS()
{
  numRegions   = 0;
  pRegionInfo  = NULL;
  pList        = NULL;
  pImage       = NULL;
}

RGS::~RGS()
{
  if(pRegionInfo)
    delete pRegionInfo;
  if(pList)
    delete pList;
}

int RGS::init(cl_uchar4**     ppImageData,
              cl_int2*        pQueuedPixels,
              RGSParamType*   pParams)
{
  /* load image pointer */
  pImage       = *ppImageData;

  /* load image dimensions */
  width        = pParams->width;
  height       = pParams->height;

  /* get number of regions */
  numRegions   = pParams->regions;

  /* initialize region information */
  pRegionInfo    = new RGSRegionType[RGS_MAX_REGIONS];

  for(cl_int i = 0; i < RGS_MAX_REGIONS; ++i)
    {
      pRegionInfo[i].count    = 0;
      pRegionInfo[i].measure  = 0.0;
    }

  /* push seeds to sorted list */
  pList        = new RGSLinkedList;
  for(cl_int i = 0; i < numRegions; ++i)
    {
      RGSLLNodeType* pNode = new RGSLLNodeType;
      pNode->index.x    = pQueuedPixels[i].x;
      pNode->index.y    = pQueuedPixels[i].y;
      pNode->pNext      = NULL;

      pList->push(pNode);

      /* classify the node */
      cl_uint imgIndex   = TWOD_TO_ONED(pNode->index.x, pNode->index.y, width);
      pImage[imgIndex].w = i + RGS_REGION_START;

      pRegionInfo[RGS_REGION_START + i].count    = 1;
      pRegionInfo[RGS_REGION_START + i].measure  = (cl_float)(pImage[imgIndex].x);

    }

  /* reset the list */
  pList->reset();

  /* define neighbors */
  int neighX[] = {1, 1, 0,-1,-1,-1,0,1};
  int neighY[] = {0,-1,-1,-1, 0, 1,1,1};

  for (cl_int i=0; i < RGS_NEIGH; ++i)
    {
      pNeigh[i].x = (cl_int)neighX[i];
      pNeigh[i].y = (cl_int)neighY[i];
    }

  return SDK_SUCCESS;
}

int RGS::growRegion()
{
  RGSLLNodeType* pNode;
  bool           stopFlag = false;

  while(!stopFlag)
    {
      /* pop a node */
      pNode = pList->pop();

      /* if a valid node, queue all its unclassified neighbors */
      if(pNode != NULL)
        {
          queueAllNeighs(pNode);
        }
      else
        {
          /* if no pixels are queued, quit */
          if(pList->length == 0)
            stopFlag = true;
          else
            {
              /* if queued list is not empty, classify its pixels */
              pList->reset();
              classify();
            }
        }
    }
  
  return SDK_SUCCESS;
}

int RGS::queueAllNeighs(RGSLLNodeType* pNode)
{
  int       qued[RGS_NEIGH];

  /* queue even neighbors */
  for(cl_int i = 0; i < RGS_NEIGH; i += 2)
    {
      if (queueNeigh(pNode, i) == SDK_SUCCESS)
        {
          qued[i] = 1;
        }
      else
        {
          qued[i] = 0;
        }
    }

  /* queue odd neighbors based on how even neighbors are queued */
  if(qued[0] && qued[2])
    queueNeigh(pNode,1);

  if(qued[2] && qued[4])
    queueNeigh(pNode,3);

  if(qued[4] && qued[6])
    queueNeigh(pNode,5);

  if(qued[6] && qued[0])
    queueNeigh(pNode,7);

  return SDK_SUCCESS;
}

int RGS::queueNeigh(RGSLLNodeType* pNode, cl_int i)
{
  cl_int2   neigh;
  cl_int    imgIndex;
  cl_uchar  neighRegionInfo;

  cl_int    status = SDK_FAILURE;

  neigh.x = pNode->index.x + pNeigh[i].x;
  neigh.y = pNode->index.y + pNeigh[i].y;

  imgIndex         = TWOD_TO_ONED(neigh.x, neigh.y, width);
  neighRegionInfo  = pImage[imgIndex].w;
  if(neighRegionInfo == RGS_REGION_UNCLASSIFIED)
    {
      /* define the pixel */
      RGSLLNodeType* pNeighNode = new RGSLLNodeType;
      pNeighNode->index         = neigh;
      pNeighNode->pNext         = NULL;
      
      /* push the pixel */
      pList->push(pNeighNode);
      
      /* mark pixel as queued. */
      pImage[imgIndex].w = RGS_REGION_QUEUED;
      
      status = SDK_SUCCESS;
    }
  
  return status;
}

int RGS::classify()
{
  RGSLLNodeType* pNode;
  cl_uint        i,r,c;
  cl_float       pixLuma;

  pNode = pList->pHead->pNext;

  while(pNode != pList->pMid)
    {
      c           = pNode->index.x;
      r           = pNode->index.y;
      i           = TWOD_TO_ONED(c,r,width);
      pixLuma     = (cl_float)(pImage[i].x);
      pImage[i].w = (cl_uchar)(minRegion(pixLuma));
      
      pNode       = pNode->pNext;
    }

  return SDK_SUCCESS;
}

int RGS::minRegion(cl_float pixLuma)
{
  cl_int    minReg;
  cl_float  regLuma;
  cl_float  minLuma;
  cl_float  diffLuma;
  cl_float  n;

  /* find minimum difference region */
  minReg       = RGS_REGION_START;
  regLuma      = pRegionInfo[RGS_REGION_START].measure;
  minLuma      = FABS(pixLuma - regLuma);
  
  for(cl_int i = 1; i < numRegions; ++i)
    {
      regLuma  = pRegionInfo[i + RGS_REGION_START].measure;
      diffLuma = FABS(pixLuma - regLuma);
      
      if(diffLuma < minLuma)
        {
          minLuma = diffLuma;
          minReg  = i + RGS_REGION_START;
        }
    }

  /* update region information */
  n               = (float)(pRegionInfo[minReg].count);
  regLuma         = pRegionInfo[minReg].measure;

  regLuma         = (n*regLuma + pixLuma)/(n + 1.0f); 

  pRegionInfo[minReg].count    += 1;  
  pRegionInfo[minReg].measure   = regLuma;

  return minReg;
}

#endif
