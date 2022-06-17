#ifndef MEMORY_ILIST_H_
#define MEMORY_ILIST_H_

//Libraries
#include <iostream>



template<class T>
class  IListElement
{
   //friend class IList;

  public:

    IListElement()
    : mNext(nullptr),
      mPrev(nullptr)
    {
    }

    IListElement( T* element ,IListElement *next, IListElement *prev)
    : mNext(next),
      mPrev(prev),
      mPointer(element)
    {
    }

    ~IListElement()
    {
//        if(mPointer != nullptr)
//        {
//            delete    mPointer;
//            mPointer = nullptr;
//        }
    }


    IListElement *GetNext() const { return mNext; }
    IListElement *GetPrev() const { return mPrev; }


    bool IsHead() const { return mPrev == 0; }
    bool IsTail() const { return mNext == 0; }


    void InsertBefore(IListElement *link)
    {
        mNext         = link;
        mPrev         = link->mPrev;
        mNext->mPrev = this;
        mPrev->mNext = this;
    }


    void InsertAfter(IListElement *link)
    {
        mNext         = link->mNext;
        mPrev         = link;
        mNext->mPrev = this;
        mPrev->mNext = this;
    }


    void Remove()
    {
        mNext->mPrev = mPrev;
        mPrev->mNext = mNext;
    }



    ///Get real value
    T *GetPointer() const;

    void SetPointer(T *pointer);

//private:

    IListElement  *mNext;
    IListElement  *mPrev;

    ///pointer value
    T *mPointer;
};


///Get real value
template< class T >
T *IListElement<T>::GetPointer() const
{
   return mPointer;
}

template< class T >
void IListElement<T>::SetPointer(T *pointer)
{
   mPointer = pointer;
}


template<class T>
class IList
{
  public:

    IList()
    : mHead(&mTail, 0),
      mTail(0, &mHead)
    {
    }

    IListElement<T> *GetHead() const { return mHead.getNext(); }
    IListElement<T> *GetTail() const { return mTail.getPrev(); }

    void AddHead(IListElement<T> *link) { link->insertAfter(&mHead); }
    void AddTail(IListElement<T> *link) { link->insertBefore(&mTail); }


//    void Add(T *Data)
//    {
//        IListElement<T> *temp=new IListElement<T>; //Выделение памяти под новый элемент структуры
//        temp->mNext=NULL;  //Указываем, что изначально по следующему адресу пусто
//        temp->SetPointer(Data);//Записываем значение в структуру

//        if (mHead!=NULL) //Если список не пуст
//        {
//            temp->mPrev=mTail; //Указываем адрес на предыдущий элемент в соотв. поле
//            mTail->mNext=temp; //Указываем адрес следующего за хвостом элемента
//            mTail=temp; //Меняем адрес хвоста
//        }
//        else //Если список пустой
//        {
//            temp->mPrev=NULL; //Предыдущий элемент указывает в пустоту
//            mHead=mTail=temp; //Голова=Хвост=тот элемент, что сейчас добавили
//        }
//    }


 private:

    IListElement<T> mHead;
    IListElement<T> mTail;
};








#endif /* MEMORY_ILIST_H_ */
