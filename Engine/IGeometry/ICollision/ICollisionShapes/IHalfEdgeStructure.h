#ifndef IHALFEDGESTRUCTURE_H
#define IHALFEDGESTRUCTURE_H

#include "../../../imaths.hpp"
#include "../../../ICommon/ISettings.h"
#include "../../../ICommon/IMemory/IPair.h"

// pair::pair example
#include <utility>      // std::pair, std::make_pair
#include <string>       // std::string
#include <iostream>     // std::cout


namespace IEngine
{


// Class List
/**
 * This class represents a simple generic list with custom memory allocator.
  */
template<typename T>
class List {

    private:

        // -------------------- Attributes -------------------- //

        /// Buffer for the list elements
        void* mBuffer;

        /// Number of elements in the list
        size_t mSize;

        /// Number of allocated elements in the list
        size_t mCapacity;


    public:

        /// Class Iterator
        /**
         * This class represents an iterator for the List
         */
        class Iterator {

            private:

                size_t mCurrentIndex;
                T* mBuffer;
                size_t mSize;

            public:

                // Iterator traits
                using value_type = T;
                using difference_type = std::ptrdiff_t;
                using pointer = T*;
                using reference = T&;
                using iterator_category = std::bidirectional_iterator_tag;

                /// Constructor
                Iterator() = default;

                /// Constructor
                Iterator(void* buffer, size_t index, size_t size)
                     :mCurrentIndex(index), mBuffer(static_cast<T*>(buffer)), mSize(size) {

                }

                /// Copy constructor
                Iterator(const Iterator& it)
                     :mCurrentIndex(it.mCurrentIndex), mBuffer(it.mBuffer), mSize(it.mSize) {

                }

                /// Deferencable
                reference operator*() const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex < mSize);
                    return mBuffer[mCurrentIndex];
                }

                /// Deferencable
                pointer operator->() const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex < mSize);
                    return &(mBuffer[mCurrentIndex]);
                }

                /// Post increment (it++)
                Iterator& operator++() {
                    assert(mCurrentIndex < mSize);
                    mCurrentIndex++;
                    return *this;
                }

                /// Pre increment (++it)
                Iterator operator++(int number) {
                    assert(mCurrentIndex < mSize);
                    Iterator tmp = *this;
                    mCurrentIndex++;
                    return tmp;
                }

                /// Post decrement (it--)
                Iterator& operator--() {
                    assert(mCurrentIndex > 0);
                    mCurrentIndex--;
                    return *this;
                }

                /// Pre decrement (--it)
                Iterator operator--(int number) {
                    assert(mCurrentIndex > 0);
                    Iterator tmp = *this;
                    mCurrentIndex--;
                    return tmp;
                }

                /// Equality operator (it == end())
                bool operator==(const Iterator& iterator) const {
                    assert(mCurrentIndex >= 0 && mCurrentIndex <= mSize);

                    // If both iterators points to the end of the list
                    if (mCurrentIndex == mSize && iterator.mCurrentIndex == iterator.mSize) {
                        return true;
                    }

                    return &(mBuffer[mCurrentIndex]) == &(iterator.mBuffer[iterator.mCurrentIndex]);
                }

                /// Inequality operator (it != end())
                bool operator!=(const Iterator& iterator) const {
                    return !(*this == iterator);
                }

                /// Frienship
                friend class List;

        };

        // -------------------- Methods -------------------- //

        /// Constructor
        List( size_t capacity = 0)
            : mBuffer(nullptr), mSize(0), mCapacity(0)
        {

            if (capacity > 0)
            {

                // Allocate memory
                reserve(capacity);
            }
        }

        /// Copy constructor
        List(const List<T>& list) : mBuffer(nullptr), mSize(0), mCapacity(0)
        {

            // All all the elements of the list to the current one
            addRange(list);
        }

        /// Destructor
        ~List()
        {

            // If elements have been allocated
            if (mCapacity > 0)
            {

                // Clear the list
                clear();

                // Release the memory allocated on the heap
               // mAllocator.release(mBuffer, mCapacity * sizeof(T));
                free(mBuffer);
            }
        }

        /// Allocate memory for a given number of elements
        void reserve(size_t capacity) {

            if (capacity <= mCapacity) return;

            // Allocate memory for the new array
            //void* newMemory = mAllocator.allocate(capacity * sizeof(T));
            void* newMemory = (void*)malloc(capacity*sizeof(T));

            if (mBuffer != nullptr) {

                if (mSize > 0) {

                    // Copy the elements to the new allocated memory location
                    T* destination = static_cast<T*>(newMemory);
                    T* items = static_cast<T*>(mBuffer);
                    std::uninitialized_copy(items, items + mSize, destination);

                    // Destruct the previous items
                    for (size_t i=0; i<mSize; i++) {
                        items[i].~T();
                    }
                }

                // Release the previously allocated memory
               // mAllocator.release(mBuffer, mCapacity * sizeof(T));
                free(mBuffer);
            }

            mBuffer = newMemory;
            assert(mBuffer != nullptr);

            mCapacity = capacity;
        }

        /// Add an element into the list
        void add(const T& element) {

            // If we need to allocate more memory
            if (mSize == mCapacity) {
                reserve(mCapacity == 0 ? 1 : mCapacity * 2);
            }

            // Use the copy-constructor to construct the element
            new (static_cast<char*>(mBuffer) + mSize * sizeof(T)) T(element);

            mSize++;
        }

        /// Try to find a given item of the list and return an iterator
        /// pointing to that element if it exists in the list. Otherwise,
        /// this method returns the end() iterator
        Iterator find(const T& element) {

            for (u32 i=0; i<mSize; i++) {
                if (element == static_cast<T*>(mBuffer)[i]) {
                    return Iterator(mBuffer, i, mSize);
                }
            }

            return end();
        }

        /// Look for an element in the list and remove it
        Iterator remove(const T& element) {
           return remove(find(element));
        }

        /// Remove an element from the list and return a iterator
        /// pointing to the element after the removed one (or end() if none)
        Iterator remove(const Iterator& it) {
           assert(it.mBuffer == mBuffer);
           return removeAt(it.mCurrentIndex);
        }

        /// Remove an element from the list at a given index and return an
        /// iterator pointing to the element after the removed one (or end() if none)
        Iterator removeAt(u32 index) {

          assert(index >= 0 && index < mSize);

          // Call the destructor
          (static_cast<T*>(mBuffer)[index]).~T();

          mSize--;

          if (index != mSize) {

              // Move the elements to fill in the empty slot
              char* dest = static_cast<char*>(mBuffer) + index * sizeof(T);
              char* src = dest + sizeof(T);
              std::memmove(static_cast<void*>(dest), static_cast<void*>(src), (mSize - index) * sizeof(T));
          }

          // Return an iterator pointing to the element after the removed one
          return Iterator(mBuffer, index, mSize);
        }

        /// Append another list at the end of the current one
        void addRange(const List<T>& list) {

            // If we need to allocate more memory
            if (mSize + list.size() > mCapacity) {

                // Allocate memory
                reserve(mSize + list.size());
            }

            // Add the elements of the list to the current one
            for(u32 i=0; i<list.size(); i++) {

                new (static_cast<char*>(mBuffer) + mSize * sizeof(T)) T(list[i]);
                mSize++;
            }
        }

        /// Clear the list
        void clear() {

            // Call the destructor of each element
            for (u32 i=0; i < mSize; i++) {
                (static_cast<T*>(mBuffer)[i]).~T();
            }

            mSize = 0;
        }

        /// Return the number of elements in the list
        size_t size() const {
            return mSize;
        }

        /// Return the capacity of the list
        size_t capacity() const {
            return mCapacity;
        }

        /// Overloaded index operator
        T& operator[](const u32 index) {
           assert(index >= 0 && index < mSize);
           return (static_cast<T*>(mBuffer)[index]);
        }

        /// Overloaded const index operator
        const T& operator[](const u32 index) const {
           assert(index >= 0 && index < mSize);
           return (static_cast<T*>(mBuffer)[index]);
        }

        /// Overloaded equality operator
        bool operator==(const List<T>& list) const {

           if (mSize != list.mSize) return false;

           T* items = static_cast<T*>(mBuffer);
            for (size_t i=0; i < mSize; i++) {
                if (items[i] != list[i]) {
                    return false;
                }
            }

            return true;
        }

        /// Overloaded not equal operator
        bool operator!=(const List<T>& list) const {

            return !((*this) == list);
        }

        /// Overloaded assignment operator
        List<T>& operator=(const List<T>& list) {

            if (this != &list) {

                // Clear all the elements
                clear();

                // Add all the elements of the list to the current one
                addRange(list);
            }

            return *this;
        }

        /// Return a begin iterator
        Iterator begin() const {
            return Iterator(mBuffer, 0, mSize);
        }

        /// Return a end iterator
        Iterator end() const {
            return Iterator(mBuffer, mSize, mSize);
        }
};

// Class HalfEdgeStructure
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangle. Note that the half-edge structure
 * is only valid if the mesh is closed (each edge has two adjacent faces).
 */
class HalfEdgeStructure {

    public:

        using VerticesPair = std::pair<u32, u32>;

        /// Edge
        struct Edge
        {
            u32 vertexIndex;       // Index of the vertex at the beginning of the edge
            u32 twinEdgeIndex;     // Index of the twin edge
            u32 faceIndex;         // Adjacent face index of the edge
            u32 nextEdgeIndex;     // Index of the next edge
        };

        /// Face
        struct Face
        {
            u32 edgeIndex;             // Index of an half-edge of the face
            List<u32> faceVertices;	// Index of the vertices of the face

            /// Constructor
            Face() : faceVertices() {}

            /// Constructor
            Face(List<u32> vertices) : faceVertices(vertices) {}
        };

        /// Vertex
        struct Vertex
        {
            u32 vertexPointIndex;  // Index of the vertex point in the origin vertex array
            u32 edgeIndex;         // Index of one edge emanting from this vertex

            /// Constructor
            Vertex(u32 vertexCoordsIndex) : vertexPointIndex(vertexCoordsIndex) { }
        };

    private:

        /// All the faces
        List<Face> mFaces;

        /// All the vertices
        List<Vertex> mVertices;

        /// All the half-edges
        List<Edge> mEdges;

    public:

        /// Constructor
        HalfEdgeStructure(u32 facesCapacity,
                          u32 verticesCapacity,
                          u32 edgesCapacity) :
                          mFaces(facesCapacity),
                          mVertices(verticesCapacity),
                          mEdges(edgesCapacity) {}

        /// Destructor
        ~HalfEdgeStructure() = default;

        /// Initialize the structure (when all vertices and faces have been added)
        void init();

        /// Add a vertex
        u32 addVertex(u32 vertexPointIndex);

        /// Add a face
        void addFace(List<u32> faceVertices);

        /// Return the number of faces
        u32 getNbFaces() const;

        /// Return the number of half-edges
        u32 getNbHalfEdges() const;

        /// Return the number of vertices
        u32 getNbVertices() const;

        /// Return a given face
        const Face& getFace(u32 index) const;

        /// Return a given edge
        const Edge& getHalfEdge(u32 index) const;

        /// Return a given vertex
        const Vertex& getVertex(u32 index) const;

};

// Add a vertex
/**
 * @param vertexPointIndex Index of the vertex in the vertex data array
 */
inline u32 HalfEdgeStructure::addVertex(u32 vertexPointIndex)
{
    Vertex vertex(vertexPointIndex);
    mVertices.add(vertex);
    return mVertices.size() - 1;
}

// Add a face
/**
 * @param faceVertices List of the vertices in a face (ordered in CCW order as seen from outside
 *                     the polyhedron
 */
inline void HalfEdgeStructure::addFace(List<u32> faceVertices)
{
    // Create a new face
    Face face(faceVertices);
    mFaces.add(face);
}

// Return the number of faces
/**
 * @return The number of faces in the polyhedron
 */
inline u32 HalfEdgeStructure::getNbFaces() const
{
    return static_cast<u32>(mFaces.size());
}

// Return the number of edges
/**
 * @return The number of edges in the polyhedron
 */
inline u32 HalfEdgeStructure::getNbHalfEdges() const
{
    return static_cast<u32>(mEdges.size());
}

// Return the number of vertices
/**
 * @return The number of vertices in the polyhedron
 */
inline u32 HalfEdgeStructure::getNbVertices() const
{
    return static_cast<u32>(mVertices.size());
}

// Return a given face
/**
 * @return A given face of the polyhedron
 */
inline const HalfEdgeStructure::Face& HalfEdgeStructure::getFace(u32 index) const
{
    assert(index < mFaces.size());
    return mFaces[index];
}

// Return a given edge
/**
 * @return A given edge of the polyhedron
 */
inline const HalfEdgeStructure::Edge& HalfEdgeStructure::getHalfEdge(u32 index) const
{
    assert(index < mEdges.size());
    return mEdges[index];
}

// Return a given vertex
/**
 * @return A given vertex of the polyhedron
 */
inline const HalfEdgeStructure::Vertex& HalfEdgeStructure::getVertex(u32 index) const
{
    assert(index < mVertices.size());
    return mVertices[index];
}

}

#endif // IHALFEDGESTRUCTURE_H
