#ifndef IMEMORY_H_ 
#define IMEMORY_H_ 


#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <algorithm>


    static unsigned int allocCalls = 0;
    static unsigned int maxAllocCalls = 0;

    static void* IAlloc(unsigned int size)
    {
        ++allocCalls;
        maxAllocCalls = std::max(maxAllocCalls, allocCalls);
        return malloc(size);
    }

    static void IFree(void* block)
    {
        return free(block);
    }


    static void *IRealloc(void* block , unsigned int size)
    {
        return realloc(block,size);
    }






#endif /* IMEMORY_H_ */
