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

#ifndef _GUI_HPP_
#define _GUI_HPP_

/********************************************************************************
* MonteCarloPIGUI                                                               *
* Class that displays GUI for monte-carlo simulation                            *
*********************************************************************************/
class MonteCarloPIGUI
{
    public:

        int numPoints;
        std::vector<Point>
        inputPoints;                     /**< Container for the random points */
        bolt::cl::device_vector<Point> boltInputPoints;
        float PIValue;

        MonteCarloPIGUI(int _numPoints=100000):
            boltInputPoints(0, Point(), CL_MEM_READ_ONLY | CL_MEM_USE_PERSISTENT_MEM_AMD)
        {
            numPoints = _numPoints;
        }

        ~MonteCarloPIGUI()
        {
            inputPoints.clear();
        }

        /**
        ***************************************************************************
        * @fn usage
        * @brief Prints help message and options for GUI related operations
        * @return 0 on success and nonzero on failure
        **************************************************************************/
        int usage();

        /**
        ***************************************************************************
        * @fn update
        * @brief Updates class members when any key is pressed on the GUI window
        * @return 0 on success and nonzero on failure
        **************************************************************************/
        int update();

        /**
        ***************************************************************************
        * @fn showGUI
        * @brief Entry point function to this class that shows GUI
        * @return 0 on success and nonzero on failure
        **************************************************************************/
        int showGUI (int argc, char **argv);
};


#endif  // #ifndef _GUI_HPP_
