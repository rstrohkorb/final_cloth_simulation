///
///  @file BVTree.cpp
///  @brief Implimentation of BVTree class

#include <numeric>
#include <ngl/Types.h>
#include "BVTree.h"

void BVTree::init(const std::vector<ngl::Vec3> _tridata)
{
    // Populate m_surfaces and m_surfaceCenters
    m_surfaces.reserve(_tridata.size()/3);
    m_surfaceCenters.reserve(_tridata.size()/3);
    createSurfaces(_tridata);
    // Create the initial i-reference list
    std::vector<size_t> surfacei;
    surfacei.resize(m_surfaceCenters.size());
    std::iota(surfacei.begin(), surfacei.end(), 0);
    // Allocate the BVTree
    allocateTree(surfacei);
}

void BVTree::updateSurfaces(const std::vector<ngl::Vec3> _tridata)
{
    for(size_t i = 0; i < _tridata.size(); i += 3)
    {
        // m_surfaces
        m_surfaces[i/3].s1 = _tridata[i];
        m_surfaces[i/3].s2 = _tridata[i + 1];
        m_surfaces[i/3].s3 = _tridata[i + 2];
        // m_surfaceCenters
        std::vector<ngl::Vec3> v = {_tridata[i], _tridata[i + 1], _tridata[i + 2]};
        m_surfaceCenters[i/3] = calcCenter(v);
    }
}

void BVTree::createSurfaces(const std::vector<ngl::Vec3> _tridata)
{
    for(size_t i = 0; i < _tridata.size(); i += 3)
    {
        // m_surfaces
        m_surfaces.push_back(Surface(_tridata[i], _tridata[i + 1], _tridata[i + 2]));
        // m_surfaceCenters
        std::vector<ngl::Vec3> v = {_tridata[i], _tridata[i + 1], _tridata[i + 2]};
        m_surfaceCenters.push_back(calcCenter(v));
    }
}

ColDetectInfo BVTree::detectCollision(BVTree &_eTree)
{
    size_t i_flying = 0;
    size_t i_environ = 0;
    return detectCollisionHelper(_eTree, i_flying, i_environ);
}

void BVTree::clear()
{
    // De-allocate everything
    m_tree.clear();
    m_surfaceCenters.clear();
    m_surfaces.clear();
}

ngl::Vec3 BVTree::calcCenter(std::vector<ngl::Vec3> _vertices)
{
    auto center = std::accumulate(_vertices.begin(), _vertices.end(), ngl::Vec3(0.0f));
    center /= _vertices.size();
    checkNearZero(center);
    return center;
}

void BVTree::updateAllBV()
{
    for(auto &s : m_tree)
    {
        updateBV(s);
    }
}

void BVTree::allocateTree(std::vector<size_t> _si)
{
    // Resize tree
    m_tree.resize(m_surfaceCenters.size() - 1);
    // Create root node
    m_tree[0] = SphereBV(_si);
    updateBV(m_tree[0]);
    // Determine dividing axis - plane normal
    ngl::Vec3 div(0.0f);
    auto var = computeVariance(_si);
    if(var.m_x > var.m_y && var.m_x > var.m_z)
    {
        div.m_x = 1.0f;
    }
    else if(var.m_y > var.m_x && var.m_y > var.m_z)
    {
        div.m_y = 1.0f;
    }
    else
    {
        div.m_z = 1.0f;
    }
    // Aquire mean - split plane point
    auto mean = calcCenter(m_surfaceCenters);
    // Set up lists for children
    std::vector<size_t> left;
    std::vector<size_t> right;
    left.reserve(_si.size()/2);
    right.reserve(_si.size()/2);
    // Sort each point by the given plane
    for(size_t i : _si)
    {
        if(sortByPlane(div, mean, m_surfaceCenters[i]) < 0.0f)
        {
            left.push_back(i);
        }
        else
        {
            right.push_back(i);
        }
    }
    // Pass off to children
    if(left.size() > 0)
    {
        allocateTreeHelper(left, 0, true);
    }
    if(right.size() > 0)
    {
        allocateTreeHelper(right, 0, false);
    }
}

void BVTree::allocateTreeHelper(std::vector<size_t> _si, size_t _parent, bool _left)
{
    // Check if the values are the same (happens with sphere)
    if(isHomogeneous(_si))
    {
        auto temp = _si[0];
        _si.clear();
        _si.push_back(temp);
    }
    // Determine index
    size_t myIndex;
    if(_left)
    {
        myIndex = (2 * _parent) + 1;
    }
    else
    {
        myIndex = (2 * _parent) + 2;
    }
    // Make tree bigger if necessary
    try
    {
        m_tree.at(myIndex);
    }
    catch (const std::out_of_range&)
    {
        m_tree.resize(myIndex + 1);
    }
    // Create node
    m_tree[myIndex] = SphereBV(_si);
    updateBV(m_tree[myIndex]);
    // BASE CASE -- leaf node
    if(_si.size() == 1)
    {
        return;
    }
    // Determine dividing axis - plane normal
    ngl::Vec3 div(0.0f);
    auto var = computeVariance(_si);
    if(var.m_x >= var.m_y && var.m_x >= var.m_z)
    {
        div.m_x = 1.0f;
    }
    else if(var.m_y >= var.m_x && var.m_y >= var.m_z)
    {
        div.m_y = 1.0f;
    }
    else
    {
        div.m_z = 1.0f;
    }
    // Aquire mean - split plane point
    ngl::Vec3 mean(0.0f);
    for(auto i : _si)
    {
        mean += m_surfaceCenters[i];
    }
    mean /= _si.size();
    checkNearZero(mean);
    // Set up lists for children
    std::vector<size_t> left;
    std::vector<size_t> right;
    left.reserve(_si.size()/2);
    right.reserve(_si.size()/2);
    // Sort each point by the given plane
    for(size_t i : _si)
    {
        if(sortByPlane(div, mean, m_surfaceCenters[i]) < 0.0f)
        {
            left.push_back(i);
        }
        else
        {
            right.push_back(i);
        }
    }
    // Pass off to children
    if(left.size() > 0)
    {
        allocateTreeHelper(left, myIndex, true);
    }
    if(right.size() > 0)
    {
        allocateTreeHelper(right, myIndex, false);
    }
}

ngl::Vec3 BVTree::computeVariance(std::vector<size_t> _si)
{
    // Grab point values
    std::vector<ngl::Vec3> pts;
    pts.reserve(_si.size());
    for(auto i : _si)
    {
        pts.push_back(m_surfaceCenters[i]);
    }
    // Acquire mean
    auto mean = calcCenter(pts);
    // Calculate variance
    ngl::Vec3 var(0.0f);
    for(auto p : pts)
    {
        var += (p - mean) * (p - mean);
    }
    var /= pts.size();
    checkNearZero(var);
    // Return
    return var;
}

float BVTree::sortByPlane(ngl::Vec3 _n, ngl::Vec3 _p, ngl::Vec3 _in)
{
    return (_n.m_x * (_in.m_x - _p.m_x)) + (_n.m_y * (_in.m_y - _p.m_y)) + (_n.m_z * (_in.m_z - _p.m_z));
}

void BVTree::updateBV(SphereBV &o_sbv)
{
    // Acquire new center
    ngl::Vec3 ncenter(0.0f);
    for(auto i : o_sbv.surfaces)
    {
        ncenter += m_surfaces[i].s1;
        ncenter += m_surfaces[i].s2;
        ncenter += m_surfaces[i].s3;
    }
    ncenter /= (o_sbv.surfaces.size() * 3);
    checkNearZero(ncenter);
    o_sbv.center = ncenter;
    // Acquire new radius
    float nradius = 0.0f;
    for(auto i : o_sbv.surfaces)
    {
        auto dist1 = (o_sbv.center - m_surfaces[i].s1).length();
        auto dist2 = (o_sbv.center - m_surfaces[i].s2).length();
        auto dist3 = (o_sbv.center - m_surfaces[i].s3).length();
        if(dist1 > nradius)
        {
            nradius = dist1;
        }
        if(dist2 > nradius)
        {
            nradius = dist2;
        }
        if(dist3 > nradius)
        {
            nradius = dist3;
        }
    }
    o_sbv.radius = nradius;
}

ColDetectInfo BVTree::detectCollisionHelper(BVTree &_eTree, const size_t _iself, const size_t _iother)
{
    ColDetectInfo collisions;
    collisions.detected = false;
    // if the two BVs intersect
    if(m_tree[_iself].intersect(_eTree.getBV(_iother)))
    {
        // Base case - we're on eTree's leaf node
        if(_eTree.getBV(_iother).surfaces.size() == 1)
        {
            // Base case - we're on this tree's leaf node
            if(m_tree[_iself].surfaces.size() == 1)
            {
                // Check for collision on these two triangles
                Surface surfaceSelf = m_surfaces[m_tree[_iself].surfaces.front()];
                Surface surfaceOther = _eTree.getSurface(_eTree.getBV(_iother).surfaces.front());
                auto triCollision = detectTriangleCollision(surfaceSelf, surfaceOther);
                // Return triangle info
                if(triCollision.first == true)
                {
                    collisions.detected = true;
                    TriDetectInfo tri = triCollision.second;
                    tri.selfTriNum = m_tree[_iself].surfaces.front();
                    tri.otherTriNum = _eTree.getBV(_iother).surfaces.front();
                    collisions.collisions.push_back(tri);
                }
            }
            else
            {
                // Push to children of flying architecture
                auto leftFly = detectCollisionHelper(_eTree, 2*_iself + 1, _iother);
                auto rightFly = detectCollisionHelper(_eTree, 2*_iself + 2, _iother);
                // Check results and add
                if(leftFly.detected == true)
                {
                    collisions.detected = true;
                    for(auto s : leftFly.collisions)
                    {
                        collisions.collisions.push_back(s);
                    }
                }
                if(rightFly.detected == true)
                {
                    collisions.detected = true;
                    for(auto s : rightFly.collisions)
                    {
                        collisions.collisions.push_back(s);
                    }
                }
            }
        }
        else
        {
            // Push to children of environemnt architecture
            auto leftEnv = detectCollisionHelper(_eTree, _iself, 2*_iother + 1); //left
            auto rightEnv = detectCollisionHelper(_eTree, _iself, 2*_iother + 2); //right
            // Check results and add
            if(leftEnv.detected == true)
            {
                collisions.detected = true;
                for(auto s : leftEnv.collisions)
                {
                    collisions.collisions.push_back(s);
                }
            }
            if(rightEnv.detected == true)
            {
                collisions.detected = true;
                for(auto s : rightEnv.collisions)
                {
                    collisions.collisions.push_back(s);
                }
            }
        }
    }
    return collisions;
}

std::pair<bool, TriDetectInfo> BVTree::detectTriangleCollision(const Surface _surface1, const Surface _surface2)
{
    /* Code for this function was modified from the following tutorial:
     * BraynzarSoft, 2015. "Triangle to Triangle Collision Detection." Available from:
     * https://www.braynzarsoft.net/viewtutorial/q16390-24-triangle-to-triangle-collision-detection
     * [Accessed 23 January 2019].
     */
    std::pair<bool, TriDetectInfo> retVal;
    retVal.first = false;
    // Step 1 - find face normal and center point of _surface1
    auto s1normal = (_surface1.s2 - _surface1.s1).cross(_surface1.s3 - _surface1.s1);
    s1normal.normalize();
    auto s1center = (_surface1.s1 + _surface1.s2 + _surface1.s3)/3;
    // Step 2 - store values for plane equation (plane of _surface1)
    float A = s1normal.m_x;
    float B = s1normal.m_y;
    float C = s1normal.m_z;
    float D = ((-A * s1center.m_x) - (B * s1center.m_y) - (C * s1center.m_z));
    // Step 3 - find 3 't' values - if it's between 0 and 1, this line intersects
    // LINE 1
    auto ep1 = (A * _surface2.s1.m_x) + (B * _surface2.s1.m_y) + (C * _surface2.s1.m_z);
    auto ep2 = (A * _surface2.s2.m_x) + (B * _surface2.s2.m_y) + (C * _surface2.s2.m_z);
    float t = -1.0f;
    if(!FCompare(ep1, ep2))
    {
        t = -(ep2 + D)/(ep1 - ep2);
    }
    if(t >= 0.0f && t <= 1.0f) //if intersect with plane
    {
        ngl::Vec3 intersect = (_surface2.s1 * t) + (_surface2.s2 * (1 - t));
        if(pointInTriangle(_surface1, intersect))
        {
            //Intersect has occured with this line on this point
            TriDetectInfo res;
            res.edgeNums.push_back(0);
            res.intersectPoints.push_back(intersect);
            retVal.first = true;
            retVal.second = res;
        }
    }
    // LINE 2
    ep1 = (A * _surface2.s2.m_x) + (B * _surface2.s2.m_y) + (C * _surface2.s2.m_z);
    ep2 = (A * _surface2.s3.m_x) + (B * _surface2.s3.m_y) + (C * _surface2.s3.m_z);
    t = -1.0f;
    if(!FCompare(ep1, ep2))
    {
        t = -(ep2 + D)/(ep1 - ep2);
    }
    if(t >= 0.0f && t <= 1.0f) // if intersect with plane
    {
        ngl::Vec3 intersect = (_surface2.s2 * t) + (_surface2.s3 * (1 - t));
        if(pointInTriangle(_surface1, intersect))
        {
            //Intersect has occured with this line on this point
            TriDetectInfo res;
            res.edgeNums.push_back(1);
            res.intersectPoints.push_back(intersect);
            retVal.first = true;
            retVal.second = res;
        }
    }
    // LINE 3
    ep1 = (A * _surface2.s3.m_x) + (B * _surface2.s3.m_y) + (C * _surface2.s3.m_z);
    ep2 = (A * _surface2.s1.m_x) + (B * _surface2.s1.m_y) + (C * _surface2.s1.m_z);
    t = -1.0f;
    if(!FCompare(ep1, ep2))
    {
        t = -(ep2 + D)/(ep1 - ep2);
    }
    if(t >= 0.0f && t <= 1.0f) // if intersect with plane
    {
        ngl::Vec3 intersect = (_surface2.s3 * t) + (_surface2.s1 * (1 - t));
        if(pointInTriangle(_surface1, intersect))
        {
            //Intersect has occured with this line on this point
            TriDetectInfo res;
            res.edgeNums.push_back(2);
            res.intersectPoints.push_back(intersect);
            retVal.first = true;
            retVal.second = res;
        }
    }
    return retVal;
}

bool BVTree::pointInTriangle(const Surface _surface, const ngl::Vec3 _point)
{
    /* Code for this function was modified from the following tutorial:
     * BraynzarSoft, 2015. "Triangle to Triangle Collision Detection." Available from:
     * https://www.braynzarsoft.net/viewtutorial/q16390-24-triangle-to-triangle-collision-detection
     * [Accessed 23 January 2019].
     */
    // Find edge lengths and surface area
    auto e1 = (_surface.s1 - _surface.s2).length();
    auto e2 = (_surface.s2 - _surface.s3).length();
    auto e3 = (_surface.s3 - _surface.s1).length();
    auto surfaceArea = triangleArea(e1, e2, e3);
    // Find 3 triangle areas using the plane intersection point
    std::vector<ngl::Vec3> vertices;
    vertices.push_back(_surface.s1);
    vertices.push_back(_surface.s2);
    vertices.push_back(_surface.s3);
    vertices.push_back(_surface.s1);
    std::vector<float> triAreas;
    triAreas.reserve(3);
    for(size_t i = 0; i < 3; ++i)
    {
        e1 = (_point - vertices[i]).length();
        e2 = (_point - vertices[i + 1]).length();
        e3 = (vertices[i] - vertices[i + 1]).length();
        triAreas.push_back(triangleArea(e1, e2, e3));
    }
    // If the 3 triAreas added together are the same as the main surface area, intersection has occurred
    return FCompare(surfaceArea, std::accumulate(triAreas.begin(), triAreas.end(), 0.0f));
}

float BVTree::triangleArea(const float _e1, const float _e2, const float _e3)
{
    /* Code for this function was modified from the following tutorial:
     * BraynzarSoft, 2015. "Triangle to Triangle Collision Detection." Available from:
     * https://www.braynzarsoft.net/viewtutorial/q16390-24-triangle-to-triangle-collision-detection
     * [Accessed 23 January 2019].
     */
    return sqrt(((_e1 + _e2 + _e3)/2.0f)*(((_e1 + _e2 + _e3)/2.0f)-_e1)
                *(((_e1 + _e2 + _e3)/2.0f)-_e2)*(((_e1 + _e2 + _e3)/2.0f)-_e3));
}

bool BVTree::isHomogeneous(const std::vector<size_t> _si)
{
    // Grab points
    std::vector<ngl::Vec3> pts;
    pts.reserve(_si.size());
    for(auto i : _si)
    {
        pts.push_back(m_surfaceCenters[i]);
    }
    // Check for differences
    size_t numDiff = 0;
    for(size_t i = 0; i < pts.size(); ++i)
    {
        for(size_t j = 0; j < pts.size(); ++j)
        {
            if(i != j)
            {
                if(pts[i] != pts[j])
                {
                    ++numDiff;
                }
            }
        }
    }
    return (numDiff == 0);
}

void BVTree::checkNearZero(ngl::Vec3 &io_v)
{
    if(FCompare(io_v.m_x, 0.0f))
    {
        io_v.m_x = 0.0f;
    }
    if(FCompare(io_v.m_y, 0.0f))
    {
        io_v.m_y = 0.0f;
    }
    if(FCompare(io_v.m_y, 0.0f))
    {
        io_v.m_y = 0.0f;
    }
}
