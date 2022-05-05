#ifndef MEMORY_ISTACK_H_
#define MEMORY_ISTACK_H_


#include "IMem.h"

//Libraries
#include <cstring>
#include <assert.h>



// Class Stack
/**
 * This class represents a simple generic stack with an initial capacity. If the number
 * of elements exceeds the capacity, the heap will be used to allocated more memory.
 */
template <typename T, unsigned int N>
class IStack
{
public:
    IStack()
    {
        mCapacity = N;
        mElements = mStackElements;
        mCount = 0;
    }

    ~IStack()
    {
        if (mElements != mStackElements)
        {
            IFree(mElements);
        }
        mElements = NULL;
    }

    const T& Top() const
    {
        assert(mCount);
        return mElements[mCount - 1];
    }

    T& Top()
    {
        assert(mCount);
        return mElements[mCount - 1];
    }

    void Push(const T& ele)
    {
        if (mCount == mCapacity)
        {
            T* oldElements = mElements;
            mCapacity *= 2;
            mElements = (T*)IAlloc(mCapacity * sizeof(T));
            memcpy(mElements, oldElements, mCount * sizeof(T));
            if (oldElements != mStackElements)
            {
                IFree(oldElements);
            }
        }
        assert(mCount < mCapacity);
        mElements[mCount] = ele;
        ++mCount;
    }

    T Pop()
    {
        assert(mCount);
        mCount--;
        return mElements[mCount];
    }

    unsigned int Count() const
    {
        return mCount;
    }

    bool IsEmpty() const
    {
        return mCount == 0;
    }
private:

    unsigned int mCapacity;
    unsigned int mCount;
    T*  mElements;
    T   mStackElements[N];
};











#endif /* MEMORY_ISTACK_H_ */
