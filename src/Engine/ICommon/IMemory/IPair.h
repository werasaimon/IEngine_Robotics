#ifndef IPAIR_H
#define IPAIR_H

// Libraries
#include <cstddef>
#include <functional>


/// This method is used to combine two hash values
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

// Class Pair
/**
 * This class represents a simple generic pair
  */
template<typename T1, typename T2>
class Pair {

    public:

        // -------------------- Attributes -------------------- //

        /// First element of the pair
        T1 first;

        /// Second element of the pair
        T2 second;

        // -------------------- Methods -------------------- //

        /// Constructor
        Pair(const T1& item1, const T2& item2) : first(item1), second(item2) {

        }

        /// Copy constructor
        Pair(const Pair<T1, T2>& pair) : first(pair.first), second(pair.second) {

        }

        /// Destructor
        ~Pair() = default;

        /// Overloaded equality operator
        bool operator==(const Pair<T1, T2>& pair) const {
            return first == pair.first && second == pair.second;
        }

        /// Overloaded not equal operator
        bool operator!=(const Pair<T1, T2>& pair) const {
            return !((*this) == pair);
        }

        /// Overloaded assignment operator
        Pair<T1, T2>& operator=(const Pair<T1, T2>& pair) {
            first = pair.first;
            second = pair.second;
            return *this;
        }
};



// Hash function for a reactphysics3d Pair
namespace std
{
  template <typename T1, typename T2> struct hash<Pair<T1, T2>>
  {
    size_t operator()(const Pair<T1, T2>& pair) const
    {
        std::size_t seed = 0;
        hash_combine<T1>(seed, pair.first);
        hash_combine<T2>(seed, pair.second);

        return seed;
    }
  };
}

#endif // IPAIR_H
