/// \file SphereBV.h
/// \brief sphere bounding volume for collision detection
/// \author Rachel Strohkorb
/// /// \version 1.0
/// \date 24/1/19 Cleaned up, added comments

#ifndef SPHEREBV_H_
#define SPHEREBV_H_

#include <vector>
#include <ngl/Vec3.h>

//--------------------------------------------------------------------------------------------------------------------------
/// @struct SphereBV "include/SphereBV.h"
/// @brief stores informaton for a spherical bounding volume
/// The goal was to eventually have an AbstractBV class which SphereBV would extend, but I didn't have time for that
//--------------------------------------------------------------------------------------------------------------------------
struct SphereBV
{
    ngl::Vec3 center = ngl::Vec3(0.0f);     //!< center of the bounding volume sphere
    float radius = 0.0f;                    //!< radius of the bounding volume sphere
    std::vector<size_t> surfaces;           //!< references to the surfaces encapsulated in this bounding volume

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default constructor
    //----------------------------------------------------------------------------------------------------------------------
    SphereBV() = default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief user constructor to easily set up SphereBV
    /// @param[in]  _surfaces references to the surfaces encapsulated in this bounding volume
    //----------------------------------------------------------------------------------------------------------------------
    SphereBV(std::vector<size_t> _surfaces) : surfaces(_surfaces) {;}
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief detects if it intersects with another SphereBV
    /// @param[in]  _sbv the SphereBV to detect an intersection with
    /// @returns  whether the two bounding volumes intersect
    //----------------------------------------------------------------------------------------------------------------------
    bool intersect(const SphereBV _sbv);
};

#endif
