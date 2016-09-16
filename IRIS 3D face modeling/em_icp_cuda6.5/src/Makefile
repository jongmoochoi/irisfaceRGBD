# /*
#   Copyright (c) 2010 Toru Tamaki
# 
#   Permission is hereby granted, free of charge, to any person
#   obtaining a copy of this software and associated documentation
#   files (the "Software"), to deal in the Software without
#   restriction, including without limitation the rights to use,
#   copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the
#   Software is furnished to do so, subject to the following
#   conditions:
# 
#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.
# 
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#   HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
#   OTHER DEALINGS IN THE SOFTWARE.
# */


#
# Check the following variables: these may differ in your environment
#
# CUDA_INSTALL_PATH := /usr/local/cuda
# CUDASDK_INSTALL_PATH := ./NVIDIA_GPU_Computing_SDK
# NVCCFLAGS := --compiler-bindir .
# LAPACKLIB := -L/usr/lib64/atlas -llapack -lptf77blas
# GLUT := -lglut


#
# Files
#

TARGET    := CUDA_EMICP_SOFTASSIGN

# Cuda source files (compiled with cudacc)
CUFILES := softassign.cu emicp.cu

# C/C++ source files (compiled with gcc / c++)
CCFILES := main.cpp findRTfromS.cpp icp.cpp emicp_cpu.cpp

# viewer
POINTCLOUDVIERER_SRC := algebra.cpp engine.cpp orbcam.cpp
CCFILES += $(POINTCLOUDVIERER_SRC)

# header file
CU_DEPS := 3dregistration.h
CC_DEPS := $(CU_DEPS)




#
# CUDA
#

CUDA_INSTALL_PATH := /usr/local/cuda
CUDASDK_INSTALL_PATH := ./NVIDIA_GPU_Computing_SDK

# Compilers
NVCC := $(CUDA_INSTALL_PATH)/bin/nvcc

NVCCFLAGS := --compiler-bindir .
# NOTE: 
# On Fedora 11, gcc4.4 doesn't work. Instead, you need gcc3.4 by
# # yum install "compat-gcc-34*"
# Also, nvcc requires a C compiler with name "gcc".
# Threfore, you need "ln -s /usr/bin/gcc34 ./gcc" and
# tell nvcc the location of it.




#
# Includes
#

# dir of cutil.h in cudasdk
CUDASDK_INCLUDES  := -I$(CUDASDK_INSTALL_PATH)/C/common/inc/

INCLUDES  += -I. -I$(CUDA_INSTALL_PATH)/include $(CUDASDK_INCLUDES)


#
# Libraries
#

# CUDA libararies
ifeq ($(emulation),1)
# emulation mode ("make emulation=1")
CUDALIB := -deviceemu -L$(CUDA_INSTALL_PATH)/lib64  -lcublasemu -lcudart -lcuda
else 
CUDALIB := -L$(CUDA_INSTALL_PATH)/lib64  -lcublas -lcudart -lcuda
endif


# library of lapack and blas
# LAPACKLIB := -llapack -lblas
# library of lapack and atlas
LAPACKLIB := -L/usr/lib64/atlas -llapack -lptf77blas

# cudasdk library
CUDASDKLIB := -L$(CUDASDK_INSTALL_PATH)/C/lib/ -lcutil_x86_64

# freeglut library
GLUTLIB := -lglut


LDFLAGS += $(CUDALIB) $(CUDASDKLIB) $(GLUTLIB) $(LAPACKLIB)




#
# Flags
#

CXXWARN_FLAGS := \
        -W -Wall -Wimplicit -Wswitch -Wformat -Wchar-subscripts \
        -Wparentheses -Wmultichar -Wtrigraphs  -Wpointer-arith \
        -Wcast-align -Wreturn-type -Wno-unused-function

ifeq ($(debug),1)
OPTFLAGS := -O0
DEBUGFLAGS := -g -ggdb
else
OPTFLAGS := -O3
endif


COMMONFLAGS += $(INCLUDES) -DUNIX  $(OPTFLAGS)

NVCCFLAGS += $(COMMONFLAGS) --use_fast_math
CXXFLAGS  += $(CXXWARN_FLAGS) $(COMMONFLAGS) 	$(DEBUGFLAGS)
CFLAGS    += $(CXXWARN_FLAGS) $(COMMONFLAGS)



ifeq ($(emulation),1)
NVCCFLAGS += -deviceemu
endif


# require openMP for emicp_cpu
CXXFLAGS += -fopenmp
LDFLAGS  += -fopenmp







#
# Rules
#


OBJS +=  $(patsubst %.cpp, %.o, $(notdir $(CCFILES)))
OBJS +=  $(patsubst %.cu,  %.o, $(notdir $(CUFILES)))

%.o : %.cu $(CU_DEPS)
	$(VERBOSE)$(NVCC) $(NVCCFLAGS) -o $@ -c $<

$(TARGET): $(OBJS) rply.o
	$(VERBOSE) $(CXX) -o $(TARGET) $(OBJS) rply.o $(LDFLAGS)

.PHONY: all clean

all: $(TARGET)

clean:
	$(RM) $(OBJS) $(TARGET)




