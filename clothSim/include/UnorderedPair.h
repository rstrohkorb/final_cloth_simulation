/// \file UnorderedPair.h
/// \brief encapsulates an unordered pair as a replacement for springs
/// \author Rachel Strohkorb
/// /// \version 1.0
/// \date 24/1/19 added comments

#ifndef UNORDEREDPAIR_H_
#define UNORDEREDPAIR_H_

#include <iostream>

//--------------------------------------------------------------------------------------------------------------------------
/// @struct UnorderedPair "include/UnorderedPair.h"
/// @brief stores references to which masspoints to which a spring is connected
//--------------------------------------------------------------------------------------------------------------------------
struct UnorderedPair
{
    size_t val0;    //!< reference to this spring's first masspoint
    size_t val1;    //!< reference to this spring's second masspoint

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief user constructor to easily set up UnorderedPair
    /// @param[in]  _val0 reference to this spring's first masspoint
    /// @param[in]  _val1 reference to this spring's second masspoint
    //----------------------------------------------------------------------------------------------------------------------
    UnorderedPair(size_t _val0, size_t _val1) : val0(_val0), val1(_val1) {;}
};
//--------------------------------------------------------------------------------------------------------------------------
/// @struct Hash_UP "include/UnorderedPair.h"
/// @brief stores a hash function for the UnorderedPair so it can be used in an unordered_map
//--------------------------------------------------------------------------------------------------------------------------
struct Hash_UP
{
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief hash function for UnorderedPair
    /// @param[in]  _up UnorderedPair to be hashed
    /// @returns  hash value for the UnorderedPair
    //----------------------------------------------------------------------------------------------------------------------
    size_t operator() (UnorderedPair const& _up) const
    {
        return (std::hash<size_t>{}(_up.val0) ^ std::hash<size_t>{}(_up.val1));
    }
};
//--------------------------------------------------------------------------------------------------------------------------
/// @struct Eq_UP "include/UnorderedPair.h"
/// @brief stores an equality operator for the UnorderedPair so it can be used in an unordered_map
//--------------------------------------------------------------------------------------------------------------------------
struct Eq_UP
{
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief equality operator for the UnorderedPair
    /// @param[in]  _lhs left-hand-side UnorderedPair
    /// @param[in]  _rhs right-hand-side UnorderedPair
    /// @returns  true if the unordered pairs contain two of the same values, no matter the order
    //----------------------------------------------------------------------------------------------------------------------
    bool operator() (UnorderedPair const& _lhs, UnorderedPair const& _rhs) const
    {
        return (((_lhs.val0 == _rhs.val0) && (_lhs.val1 == _rhs.val1)) ||
                ((_lhs.val0 == _rhs.val1) && (_lhs.val1 == _rhs.val0)));
    }
};

#endif
