/// \file BVTree.h
/// \brief Bounding Volumes Tree implimentation
/// \author Rachel Strohkorb
/// /// \version 1.0
/// \date 24/1/19 Finished up commenting, cleaned up code

#ifndef BVTREE_H_
#define BVTREE_H_

#include <vector>
#include "SphereBV.h"

//--------------------------------------------------------------------------------------------------------------------------
/// @struct Surface "include/BVTree.h"
/// @brief stores vertex information for a triangle contained in a bounding volume
//--------------------------------------------------------------------------------------------------------------------------
struct Surface
{
    ngl::Vec3 s1;   //!< first vertex
    ngl::Vec3 s2;   //!< second vertex
    ngl::Vec3 s3;   //!< third vertex
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief user constructor to easily set up Surface
    /// @param[in]  _s1 first vertex
    /// @param[in]  _s2 second vertex
    /// @param[in]  _s3 third vertex
    //----------------------------------------------------------------------------------------------------------------------
    Surface(ngl::Vec3 _s1, ngl::Vec3 _s2, ngl::Vec3 _s3) : s1(_s1), s2(_s2), s3(_s3) {;}
};
//--------------------------------------------------------------------------------------------------------------------------
/// @struct TriDetectInfo "include/BVTree.h"
/// @brief stores information on triangle collisions for a collision response system to process
//--------------------------------------------------------------------------------------------------------------------------
struct TriDetectInfo
{
    size_t selfTriNum;                      //!< number reference to this triangle
    size_t otherTriNum;                     //!< number reference to the other triangle
    std::vector<size_t> edgeNums;           //!< numbers of the edges of the other triangle that intersected this one
    std::vector<ngl::Vec3> intersectPoints; //!< points at which the other triangle intersected this one
};
//--------------------------------------------------------------------------------------------------------------------------
/// @struct ColDetectInfo "include/BVTree.h"
/// @brief stores all the triangle collisions in a given object
//--------------------------------------------------------------------------------------------------------------------------
struct ColDetectInfo
{
    bool detected;                          //!< whether or not any triangle collisions were detected
    std::vector<TriDetectInfo> collisions;  //!< triangle collisions that occured in this object
};
//--------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------
/// @class BVTree "include/BVTree.h"
/// @brief Bounding volume tree for collision detectiong between objects
//--------------------------------------------------------------------------------------------------------------------------
class BVTree
{
public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default constructor
    //----------------------------------------------------------------------------------------------------------------------
    BVTree()=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief destructor
    //----------------------------------------------------------------------------------------------------------------------
    ~BVTree()=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief copy constructor
    //----------------------------------------------------------------------------------------------------------------------
    BVTree(const BVTree &)=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief move constructor
    //----------------------------------------------------------------------------------------------------------------------
    BVTree(BVTree &&)=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief assignment operator
    //----------------------------------------------------------------------------------------------------------------------
    BVTree & operator = (const BVTree &) = default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief move assignment operator
    //----------------------------------------------------------------------------------------------------------------------
    BVTree & operator = (BVTree &&) = default;
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns total number of surfaces
    /// @returns  number of surfaces encapsulated in the BVTree
    //----------------------------------------------------------------------------------------------------------------------
    size_t numSurfaces() const { return m_surfaceCenters.size(); }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns current number nodes in the tree
    /// @returns  number of nodes in this tree
    //----------------------------------------------------------------------------------------------------------------------
    size_t numNodes() const { return m_tree.size(); }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns the bounding volume attached to the given index
    /// @param[in]  _i index at which the desired BV resides
    /// @returns  the bounding volume at the given index
    //----------------------------------------------------------------------------------------------------------------------
    SphereBV getBV (const size_t _i) const { return m_tree[_i]; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns the surface attached to the given index
    /// @param[in]  _i index at which the desired surface resides
    /// @returns  the surface at the given index
    //----------------------------------------------------------------------------------------------------------------------
    Surface getSurface(const size_t _i) const { return m_surfaces[_i]; }
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief initializes the BVTree to the given triangle data
    /// @param[in]  _tridata triangle data packed by 3's in a list of vectors
    //----------------------------------------------------------------------------------------------------------------------
    void init(const std::vector<ngl::Vec3> _tridata);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief updates all surfaces to the given vertex information
    /// @param[in]  _tridata triangle data packed by 3's in a list of vectors
    //----------------------------------------------------------------------------------------------------------------------
    void updateSurfaces(const std::vector<ngl::Vec3> _tridata);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief updates all BoundingVolumes to the current surface information
    //----------------------------------------------------------------------------------------------------------------------
    void updateAllBV();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief detects if there is a collision with the input tree
    /// @param[in]  _eTree BVTree of another object in the scene
    /// @returns  the information on triangle collision detections that occured
    //----------------------------------------------------------------------------------------------------------------------
    ColDetectInfo detectCollision(BVTree &_eTree);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief clears out all the data currently stored in this BVTree
    //----------------------------------------------------------------------------------------------------------------------
    void clear();
    //----------------------------------------------------------------------------------------------------------------------

private:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief creates surfaces based on the given vertex information
    /// @param[in]  _tridata triangle data packed by 3's in a list of vectors
    //----------------------------------------------------------------------------------------------------------------------
    void createSurfaces(const std::vector<ngl::Vec3> _tridata);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief calculates the average of the vertices in the input list
    /// @param[in]  _vertices whose center is to be found
    /// @returns  the center of the vertices
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 calcCenter(std::vector<ngl::Vec3> _vertices);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief creates and initializes the root node of the tree
    /// @param[in]  _si a list of indices referencing surfaces contained in this bounding volume
    //----------------------------------------------------------------------------------------------------------------------
    void allocateTree(std::vector<size_t> _si);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief recursively creates and initializes the non-root nodes of the tree
    /// @param[in]  _si a list of indices referencing surfaces contained in this bounding volume
    /// @param[in]  _parent the index of this node's parent
    /// @param[in]  _left whether or not this is the left child of the parent
    //----------------------------------------------------------------------------------------------------------------------
    void allocateTreeHelper(std::vector<size_t> _si, size_t _parent, bool _left);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief computes the variance of the surface centers referenced by _si
    /// @param[in]  _si a list of indices referencing surfaces
    /// @returns  the variance of the surface centers referenced by _si
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 computeVariance(std::vector<size_t> _si);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief computes the distance _in lies from the plane defined by _n and _p
    /// @param[in]  _n the normal of the plane
    /// @param[in]  _p the point that defines the position of the plane
    /// @param[in]  _in the point we wish to find on either side of the defined plane
    /// @returns  the distance away from the plane _in lies
    //----------------------------------------------------------------------------------------------------------------------
    float sortByPlane(ngl::Vec3 _n, ngl::Vec3 _p, ngl::Vec3 _in);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief updates the bounding volume based on current surface information
    /// @param[out]  o_sbv the bounding volume to be updated
    //----------------------------------------------------------------------------------------------------------------------
    void updateBV(SphereBV &o_sbv);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief recursively searches this and the input tree for collisions between their respective triangles
    /// @param[in]  _eTree BVTree of another object in the scene
    /// @param[in]  _iself the current index being searched of this tree
    /// @param[in]  _iother the current index being searched of the other object's tree
    /// @returns  the information on triangle collision detections that occured
    //----------------------------------------------------------------------------------------------------------------------
    ColDetectInfo detectCollisionHelper(BVTree &_eTree, const size_t _iself, const size_t _iother);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief detects if a collision occured between two triangles
    /// @param[in]  _surface1 the triangle from this object
    /// @param[in]  _surface2 the triangle from the other object
    /// @returns  whether a collision occured, and the information on the collision if it did occur
    //----------------------------------------------------------------------------------------------------------------------
    std::pair<bool, TriDetectInfo> detectTriangleCollision(const Surface _surface1, const Surface _surface2);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief returns whether or not a givin point intersects with the plane created by the given triangle
    /// @param[in]  _surface a triangle that defines a plane
    /// @param[in]  _point a point that could intersect with the given triangle
    /// @returns  whether this point intersects with the plane defined by the given triangle
    //----------------------------------------------------------------------------------------------------------------------
    bool pointInTriangle(const Surface _surface, const ngl::Vec3 _point);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief computes the area of a triangle
    /// @param[in]  _e1 first vertex of the triangle
    /// @param[in]  _e2 second vertex of the triangle
    /// @param[in]  _e3 third vertex of the triangle
    /// @returns  the area of the triangle
    //----------------------------------------------------------------------------------------------------------------------
    float triangleArea(const float _e1, const float _e2, const float _e3);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief returns whether or not the input list references the same values
    /// @param[in]  _si a list of indices referencing surfaces
    /// @returns  whether or not all items in the input list reference the same values
    //----------------------------------------------------------------------------------------------------------------------
    bool isHomogeneous(const std::vector<size_t> _si);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief checks if the float values in v are too close to zero and sets them to zero if they are too close
    /// this was implimented because there were massive errors caused by near-zero floating points
    /// @param[in/out]  io_v a vertex to be checked for near-zero values
    //----------------------------------------------------------------------------------------------------------------------
    void checkNearZero(ngl::Vec3 &io_v);
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// MEMBER VARIABLES
    //----------------------------------------------------------------------------------------------------------------------
    std::vector<SphereBV> m_tree;               //!< list containing the tree nodes
    std::vector<ngl::Vec3> m_surfaceCenters;    //!< centers of the surfaces encapsulated by the bounding volumes
    std::vector<Surface> m_surfaces;            //!< surface vertex data for each triangle
};

#endif
