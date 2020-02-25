/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
#define _OCL_CODE_
#include "RegionGrowingConst.hpp"

/* 
 * program scope global variables
 */

volatile __global atomic_uint      queueCount;
volatile __global atomic_uint      regSumCount[RGS_MAX_REGIONS];
volatile __global atomic_uint      regSumLuma[RGS_MAX_REGIONS];
volatile __global float            regAvgLuma[RGS_MAX_REGIONS];
volatile __global uint             regAvgCount[RGS_MAX_REGIONS];

/*
 * kernel definitions
 */

__kernel void queue_neigh(__global uchar4*           pImage,
                          __global int2*             pNodes,
                          __global RGSParamType*     pParams);

__kernel void que_classify(__global  uchar4*           pImage,
			   __global  int2*             pNodes,
			   __global  RGSParamType*     pParams);

__kernel void classify(__global  uchar4*           pImage,
                       __global  int2*             pNodes,
                       __global  RGSParamType*     pParams);

__kernel void grow_region(__global uchar4*           pInputImage,
                          __global int2*             pNodes,
                          __global RGSParamType*     pParams);
						  
__kernel void init_kernel(__global uchar4*         pImage,
                          __global int2*           pNodes,
                          __global RGSParamType*   pParams);


/*
 * queue_neigh:
 * takes already queued and classified pixels in pNodes and enqueues
 * enqueues their unclassified neighbors  for classification.
 * enques classify kernel for enqueued neighbors' classification.
 */

__kernel void queue_neigh(__global uchar4*           pImage,
                          __global int2*             pNodes,
                          __global RGSParamType*     pParams)
{

  int   nx[RGS_NEIGH]  = {1, 1, 0,-1,-1,-1,0,1};
  int   ny[RGS_NEIGH]  = {0,-1,-1,-1, 0, 1,1,1};

  int2  qn[RGS_NEIGH];
  uchar qued  = 0;
  int   count = 0; 

  uint  width     = pParams->width;
  uint  ping      = pParams->eoffset;
  uint  pong      = pParams->ooffset;
  uint  pixqueued = pParams->queued;
   
  uint  lid   = get_local_id(0);   
  uint  gid   = get_global_id(0);
  bool ret = false;

  /* one wi for a queued pixel */
  if(gid < pixqueued)
    {
      uint  qi     = gid + ping;
      int2  pi2d   = pNodes[qi]; 
      
      /* queue even neighbors */
      for(uint n = 0; n < RGS_NEIGH; n += 2)
        {
          int2 ni2d;
          int  ni1d;
          uint pw;

          ni2d.x = pi2d.x + nx[n];
          ni2d.y = pi2d.y + ny[n];

          ni1d   = TWOD_TO_ONED(ni2d.x,ni2d.y, width);
          pw     = pImage[ni1d].w;
          if(pw == RGS_REGION_UNCLASSIFIED)
            {
              qn[count].x  = ni2d.x;
              qn[count].y  = ni2d.y;
              
			  uint   wwi = (uint)(pImage[ni1d]);
			  uchar4 ww  = pImage[ni1d];
			  if(ww.w != RGS_REGION_QUEUED)
			  {
				  ww.w       = RGS_REGION_QUEUED;
				  uint   wwn = (uint)(ww);
				  ret = atomic_compare_exchange_strong((atomic_uint *)(&(pImage[ni1d])),
								 &wwi,
								 wwn);
				  if (ret)
				  {
				    count++;
					qued |= (uchar)(1 << (uchar)n);
				  }				  
			  }
            }
        }
      
      /* based on enqueuing of even neighbors, queue odd neighbors */
      if((qued & (1 << 0)) && (qued & (1 << 2)))
        {
          int2 ni2d;
          int  ni1d;
          uint pw;

          ni2d.x = pi2d.x + nx[1];
          ni2d.y = pi2d.y + ny[1];

          ni1d   = TWOD_TO_ONED(ni2d.x,ni2d.y, width);
          pw     = pImage[ni1d].w;
          
          if(pw == RGS_REGION_UNCLASSIFIED)
            {
              qn[count].x    = ni2d.x;
              qn[count].y    = ni2d.y;
              
			  uint   wwi = (uint)(pImage[ni1d]);
			  uchar4 ww  = pImage[ni1d];
			  if(ww.w != RGS_REGION_QUEUED)
			  {
				  ww.w       = RGS_REGION_QUEUED;
				  uint   wwn = (uint)(ww);
				  ret = atomic_compare_exchange_strong((atomic_uint *)(&(pImage[ni1d])),
								 &wwi,
								 wwn);
				  count = (ret) ? count+1: count;
			  }
            }
        }

      if((qued & (1 << 2)) && (qued & (1 << 4)))
        {
          int2 ni2d;
          int  ni1d;
          uint pw;

          ni2d.x = pi2d.x + nx[3];
          ni2d.y = pi2d.y + ny[3];

          ni1d   = TWOD_TO_ONED(ni2d.x,ni2d.y, width);
          pw     = pImage[ni1d].w;
          
          if(pw == RGS_REGION_UNCLASSIFIED)
            {
              qn[count].x    = ni2d.x;
              qn[count].y    = ni2d.y;
              
			  uint   wwi = (uint)(pImage[ni1d]);
			  uchar4 ww  = pImage[ni1d];
			  if(ww.w != RGS_REGION_QUEUED)
			  {
				  ww.w       = RGS_REGION_QUEUED;
				  uint   wwn = (uint)(ww);
				  ret = atomic_compare_exchange_strong((atomic_uint *)(&(pImage[ni1d])),
								 &wwi,
								 wwn);
				  count = (ret) ? count+1: count;
			  }
            }
        }

      if((qued & (1 << 4)) && (qued & (1 << 6)))
        {
          int2 ni2d;
          int  ni1d;
          uint pw;

          ni2d.x = pi2d.x + nx[5];
          ni2d.y = pi2d.y + ny[5];

          ni1d   = TWOD_TO_ONED(ni2d.x,ni2d.y, width);
          pw     = pImage[ni1d].w;
          
          if(pw == RGS_REGION_UNCLASSIFIED)
            {
              qn[count].x    = ni2d.x;
              qn[count].y    = ni2d.y;
             
			  uint   wwi = (uint)(pImage[ni1d]);
			  uchar4 ww  = pImage[ni1d];
			  if(ww.w != RGS_REGION_QUEUED)
			  {
				  ww.w       = RGS_REGION_QUEUED;
				  uint   wwn = (uint)(ww);
				  ret = atomic_compare_exchange_strong((atomic_uint *)(&(pImage[ni1d])),
								 &wwi,
								 wwn);
				  count = (ret) ? count+1: count;
			  }
            }
        }

      if((qued & (1 << 6)) && (qued & (1 << 0)))
        {
          int2 ni2d;
          int  ni1d;
          uint pw;

          ni2d.x = pi2d.x + nx[7];
          ni2d.y = pi2d.y + ny[7];

          ni1d   = TWOD_TO_ONED(ni2d.x,ni2d.y, width);
          pw     = pImage[ni1d].w;
          
          if(pw == RGS_REGION_UNCLASSIFIED)
            {
              qn[count].x    = ni2d.x;
              qn[count].y    = ni2d.y;
              
			  uint   wwi = (uint)(pImage[ni1d]);
			  uchar4 ww  = pImage[ni1d];
			  if(ww.w != RGS_REGION_QUEUED)
			  {
				  ww.w       = RGS_REGION_QUEUED;
				  uint   wwn = (uint)(ww);
				  ret = atomic_compare_exchange_strong((atomic_uint *)(&(pImage[ni1d])),
								 &wwi,
								 wwn);
				  count = (ret) ? count+1: count;
			  }
            }
        }
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);

  /* find totally queued pixel by this work-group */
  uint tcount = work_group_reduce_add(count);

  /* find relative index where the pixels of this wi should go in 
     global queue */

  uint lcount = work_group_scan_exclusive_add(count);

  /* count all the pixels queued by each work group. also find index in
     global queue where queued pixel by this work-group should go */

  uint wgindex;
  if(lid == 0)
    {
      uint tmpQueueCount;
      bool done    = false;

      while(!done)
        {
          tmpQueueCount = atomic_load(&queueCount);
          done          = atomic_compare_exchange_strong(&queueCount,
                                                         &tmpQueueCount,
                                                         tmpQueueCount + tcount); 
        }

      wgindex      = tmpQueueCount;
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);

  /* announce index for work-group in global queue to all wi */
  wgindex = work_group_broadcast(wgindex,0);

  uint windex = wgindex + lcount;

  /* finally copy queued neighbors to global queue */
  if(gid < pixqueued)
    {
      uint qi           = windex + pong;
      for(uint i= 0; i < count; ++i)
        {
          pNodes[qi +i].x = qn[i].x;
          pNodes[qi +i].y = qn[i].y;
        }
    }

  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);

  /* enqueue classify kernel via a dummy kernel */
  if(gid == 0)
    {
      if(!(pParams->hostq))
	{
	  queue_t   default_queue = get_default_queue();
	  size_t    global_size   = 64;
	  ndrange_t ndrange       = ndrange_1D(global_size);
	  void      (^fun_blk)(void) = ^{que_classify(pImage,pNodes,pParams);};
	  int status = enqueue_kernel(default_queue,
				      CLK_ENQUEUE_FLAGS_WAIT_KERNEL,
				      ndrange,
				      fun_blk);
	}
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      
}

/*
 * que_classify:
 * dummy kernel between queue_neigh and classify. this kernel is inserted
 * because classify kernel could not be launched unless global variable
 * queueCount does not contain the number of pixels queued by queue_neigh.
 * this requires that the last work-group of queue_neigh enqueue classify
 * kernel. however last work-group of queue_neigh is difficult the determine.
 * 
 */

__kernel void que_classify(__global  uchar4*           pImage,
			   __global  int2*             pNodes,
			   __global  RGSParamType*     pParams)
{

  uint gid = get_global_id(0);
  if(gid == 0)
    {
      if(!(pParams->hostq))
	{
	  uint      queued        = atomic_load(&queueCount);
	  queue_t   default_queue = get_default_queue();
	  size_t    local_size    = 64;
	  size_t    global_size   = queued;
	  ndrange_t ndrange       = ndrange_1D(global_size);
	  void      (^fun_blk)(void) = ^{classify(pImage,pNodes,pParams);};
	  int status = enqueue_kernel(default_queue,
				      CLK_ENQUEUE_FLAGS_WAIT_KERNEL,
				      ndrange,
				      fun_blk);
	}
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      

}

/*
 * classify:
 * classifies all the pixels enqueued by queue_neigh into one of the regions.
 * the region is decided by finding minimum difference of queued pixel's
 * luma with average luma of a region.
 * again enqueues grow_region kernel for next iteration.
 */

__kernel void classify(__global  uchar4*           pImage,
                       __global  int2*             pNodes,
                       __global  RGSParamType*     pParams)
{

  uint gid     = get_global_id(0);
  uint lid     = get_local_id(0);
  uint queued  = atomic_load(&queueCount);    
  uint offset  = pParams->ooffset;            
  uint width   = pParams->width;
  uint regions = pParams->regions;

  uint   pixLuma = 0;
  uint   pixReg  = RGS_REGION_BOUNDARY;
  float  diffLuma, regLuma, minLuma;
  if(gid < queued)
    {
      /*find closest region */
      int2 pi2d = pNodes[gid +offset];
      int  pi1d = TWOD_TO_ONED(pi2d.x,pi2d.y,width);
      
      pixLuma   = pImage[pi1d].x;
      regLuma   = regAvgLuma[RGS_REGION_START];

      minLuma   = FABS((float)pixLuma -regLuma);
      pixReg    = RGS_REGION_START;

      for (uint i = 1; i < regions; ++i)
        {
          regLuma   = regAvgLuma[RGS_REGION_START +i];
          diffLuma  = FABS((float)pixLuma - regLuma);
          if(diffLuma < minLuma)
            {
              minLuma = diffLuma;
              pixReg  = RGS_REGION_START +i;
            }
        }
      
      //classify
      pImage[pi1d].w  = pixReg;
    }
  work_group_barrier(CLK_LOCAL_MEM_FENCE|CLK_GLOBAL_MEM_FENCE);

  //update region info
  for(uint i = RGS_REGION_START; i < regions + RGS_REGION_START; ++i)
    {
      uint   wgCount = 0;
      uint   wgLuma  = 0;

      uint   q    = (pixReg == i) ? 1:0;
      uint   luma = q ? pixLuma : 0;

      wgLuma  = work_group_reduce_add(luma);
      wgCount = work_group_reduce_add(q);

      work_group_barrier(CLK_LOCAL_MEM_FENCE|CLK_GLOBAL_MEM_FENCE);

      if(lid == 0)
        {
          //atomically add count
          bool done = false;
          while(!done)
            {
              uint tmpRegSumCount = atomic_load(&(regSumCount[i]));
              done = atomic_compare_exchange_strong(&(regSumCount[i]),
                                                    &tmpRegSumCount,
                                                    tmpRegSumCount + wgCount);
            }
          
          //atomically add luma
          done = false;
          while(!done)
            {
              uint tmpRegSumLuma = atomic_load(&(regSumLuma[i]));
              done = atomic_compare_exchange_strong(&(regSumLuma[i]),
                                                    &tmpRegSumLuma,
                                                    tmpRegSumLuma + wgLuma);
            }
        }

      work_group_barrier(CLK_LOCAL_MEM_FENCE|CLK_GLOBAL_MEM_FENCE);
    }

  if(gid == 0)
    {
      if(!(pParams->hostq))
	{
	  queue_t   default_queue = get_default_queue();
	  size_t    global_size   = 64;
	  size_t    local_size   = 64;
	  ndrange_t ndrange       = ndrange_1D(global_size, local_size);
	  void      (^fun_blk)(void) = ^{grow_region(pImage,pNodes,pParams);};
	  int status = enqueue_kernel(default_queue,
				      CLK_ENQUEUE_FLAGS_WAIT_KERNEL,
				      ndrange,
				      fun_blk);
	}
      
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      
}

/*
 * grow_region:
 * top level region growing kernel. performs initialization formalities for
 * each iteration.
 * enqueues queue_neigh kernel.
 * 
 * queue_neigh kernel in turn enqueued classify kernel, which in turn again
 * enqueues grow_region kernel completing one iteration.
 *
 * grow_region->queue_neigh->clasify->grow_region
 *
 * iterations are terminated if there are no more pixels left to classify.
 *
 */

__kernel void grow_region(__global uchar4*           pImage,
                          __global int2*             pNodes,
                          __global RGSParamType*     pParams)
{
  uint lid = get_local_id(0);
  uint gid = get_global_id(0);

  if(gid == 0)
    {
      int tmp          = pParams->eoffset;
      pParams->eoffset = pParams->ooffset;
      pParams->ooffset = tmp;
      
      pParams->queued  = atomic_load(&queueCount);
      atomic_store(&queueCount,0);
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      
  
  //compute region average
  if(gid < RGS_MAX_REGIONS)
    {
      float sumLuma   = (float)atomic_load(&(regSumLuma[lid]));
      uint  sumCount  = atomic_load(&(regSumCount[lid]));
      float avgLuma   = regAvgLuma[lid];
      uint  avgCount  = regAvgCount[lid];
      
      avgLuma   = (float)(avgCount)*avgLuma + sumLuma;
      avgCount += sumCount;
      avgLuma  /= (float)(avgCount);
      
      regAvgLuma[lid]  = avgLuma;
      regAvgCount[lid] = avgCount;
      
      atomic_store(&(regSumLuma[lid]),0);
      atomic_store(&(regSumCount[lid]),0);
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      
  
  if(gid == 0)
    {
      if(!(pParams->hostq))
	{
	  uint      queued = pParams->queued;
	  if(queued > 0)
	    {
	      uint      wgn, res;
	      queue_t   default_queue = get_default_queue();
	      size_t    global_size   = queued;
		  size_t    local_size   = 64;
	      ndrange_t ndrange       = ndrange_1D(global_size, local_size);
	      void      (^fun_blk)(void) = ^{queue_neigh(pImage,pNodes,pParams);};
	      int status = enqueue_kernel(default_queue,
					  CLK_ENQUEUE_FLAGS_WAIT_KERNEL,
					  ndrange,
					  fun_blk);
	    }
	}
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      

}

/*
 * init_kernel:
 * initializes various program scope variables and clasifies the seed
 * pixels.
 */

__kernel void init_kernel(__global uchar4*         pImage,
                          __global int2*           pNodes,
                          __global RGSParamType*   pParams)
{
  int gid      = get_global_id(0);

  uint regions  = pParams->regions;
  uint width    = pParams->width;
  uint height   = pParams->height;
  uint queued   = pParams->queued;

  if(gid < RGS_MAX_REGIONS)
    {
      regAvgLuma[gid]  = 0.0;
      regAvgCount[gid] = 0;

      atomic_store(&(regSumLuma[gid]),0);
      atomic_store(&(regSumCount[gid]),1);
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      

  //classify seed pixels
  if(gid < regions)
    {
      int2 i2d = pNodes[gid];
      int  i1d = TWOD_TO_ONED(i2d.x, i2d.y, width);
      int  reg = gid + RGS_REGION_START;

      pImage[i1d].w            = reg; 

      //update region info
      uint pixLuma = pImage[i1d].x;
      atomic_store(&(regSumLuma[reg]),pixLuma);
      atomic_store(&(regSumCount[reg]),1);
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      

  //initialize atomic counters
  if(gid == 0)
    {
      int tmp          = pParams->eoffset;
      pParams->eoffset = pParams->ooffset;
      pParams->ooffset = tmp;

      atomic_store(&queueCount,queued);
    }
  work_group_barrier(CLK_GLOBAL_MEM_FENCE|CLK_LOCAL_MEM_FENCE);      

}


