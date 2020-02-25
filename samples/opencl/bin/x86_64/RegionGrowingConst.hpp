#ifndef _REGION_GROWING_CONST_
#define _REGION_GROWING_CONST_

#define  RGS_REGION_UNCLASSIFIED    (255)
#define  RGS_REGION_QUEUED          (250)
#define  RGS_REGION_BOUNDARY        (0)
#define  RGS_REGION_START           (1)
#define  RGS_REGION_STOP            (200)
#define  RGS_MAX_REGIONS            (200) 
#define  RGS_NEIGH                  (8)
#define  RGS_ACCEPTABLE_MISMATCH    (2.0)
#define  RGS_LOCAL_THREADS          64
#define  RGS_LOCAL_THREADS_2D       4


/* macros */
#define  FABS(x)                    (((x) > 0.0) ? (x):-(x))
#define  CLAMP(x)                   (((x) > 255)?(255):(((x) < 0)?(0):(x))) 
#define  TWOD_TO_ONED(x,y,p)        ((y)*(p) + (x))

#ifndef _OCL_CODE_

/* node for linked list */
typedef struct RGSLLNode
{
  cl_int2                  index;
  struct RGSLLNode*        pNext;
} RGSLLNodeType;

/* region information */
typedef struct RGSRegion
{
  cl_uint        count;
  cl_float       measure;
} RGSRegionType;

// parameters for GPU implementation
typedef struct RGSParams
{
  cl_uint           width;     //image width
  cl_uint           height;    //image height
  cl_uint           regions;   //number of regions image to be segmented
  cl_uint           eoffset;   //ping offset of node buffer
  cl_uint           ooffset;   //pong offset of the node buffer
  cl_uint           queued;    //number of pixels queued
  cl_int            hostq;     //flag to indicate host enqueue enabled or disabled.
} RGSParamType;

#else

// node for linked list
typedef struct RGSLLNode
{
  int2                  index;
  uchar                 region;
  float                 measure;
  struct RGSLLNode*     pNext;
} RGSLLNodeType;

// region information
typedef struct RGSRegion
{
  uint        count;
  float       measure;
} RGSRegionType;

// parameters for GPU implementation
typedef struct RGSParams
{
  uint           width;
  uint           height;
  uint           regions;
  uint           eoffset;
  uint           ooffset;
  uint           queued;
  bool           hostq;
} RGSParamType;

#endif

#endif
