/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•       Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•       Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#define BIN_ONE              0.0
#define BIN_TWO              0.9

/**
 *  binarizeDepth kernel 
 *  @brief    Binarization of a depth Image
 *  @param    image  a 2d depth image, used as input as well as output
 *  @param    threshold 
*/


__kernel void binarizeDepth(__read_write image2d_depth_t image,
                            float                        threshold)
{
  // Store each work-item's unique row and column
  int2 coord = (int2)(get_global_id(0), get_global_id(1));

  //image sampler
  sampler_t imageSampler = CLK_NORMALIZED_COORDS_FALSE | 
                           CLK_ADDRESS_CLAMP | 
                           CLK_FILTER_NEAREST; 
  
  // Read pixel from image
  float temp = read_imagef(image, imageSampler, coord);

  // Perform Image Thresholding on each pixel
  if(temp < threshold)
    temp = BIN_ONE;
  else
    temp = BIN_TWO;
  
  // Write modified pixel back to original image
  write_imagef(image, coord, temp);
}

