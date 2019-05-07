/// \file Cloth.h
/// \brief a small "cloth" object consisting of two mass points
/// \author Rachel Strohkorb
/// /// \version 1.0
/// \date 24/1/19 Finalized documentation/comments, got ready to turn it in
/// /// \todo eventually make collision detection less slow

#ifndef CLOTH_H_
#define CLOTH_H_

#include <vector>
#include <unordered_map>
#include "MassPoint.h"
#include "UnorderedPair.h"
#include "BVTree.h"

//--------------------------------------------------------------------------------------------------------------------------
/// @class Cloth "include/Cloth.h"
/// @brief A simple cloth object based on the Mass-Spring model
//--------------------------------------------------------------------------------------------------------------------------
class Cloth
{
public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default constructor
    //----------------------------------------------------------------------------------------------------------------------
    Cloth()=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default destructor, no raw pointers used
    //----------------------------------------------------------------------------------------------------------------------
    ~Cloth()=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief copy constructor
    //----------------------------------------------------------------------------------------------------------------------
    Cloth(const Cloth &)=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief move constructor
    //----------------------------------------------------------------------------------------------------------------------
    Cloth(Cloth &&)=default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief default assignment operator
    //----------------------------------------------------------------------------------------------------------------------
    Cloth & operator = (const Cloth &) = default;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief move assignment operator
    //----------------------------------------------------------------------------------------------------------------------
    Cloth & operator = (Cloth &&) = default;
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief user ctor - set the important variables needed to build the cloth
    /// @param[in]  _mass the mass given to each mass point
    /// @param[in]  _k the spring constant given to the structural springs
    /// @param[in]  _damping the damping coefficient (should be very close or equal to _k)
    /// @param[in]  _collision whether or not collision detection is being used
    //----------------------------------------------------------------------------------------------------------------------
    Cloth(float _mass, float _k, float _damping, bool _collision = false) :
        m_mass(_mass), m_k(_k), m_damping(_damping), m_collision(_collision) {;}
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns current number of masses
    /// @returns  number of masspoints in this cloth
    //----------------------------------------------------------------------------------------------------------------------
    size_t numMasses() const { return m_mspts.size(); }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns current number of springs
    /// @returns  number of springs in this cloth
    //----------------------------------------------------------------------------------------------------------------------
    size_t numSprings() const { return m_springs.size(); }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns height value of cloth
    /// @returns  height of this cloth
    //----------------------------------------------------------------------------------------------------------------------
    size_t height() const { return m_height; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns width value of cloth
    /// @returns  width of this cloth
    //----------------------------------------------------------------------------------------------------------------------
    size_t width() const { return m_width; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns mass value for each masspoint
    /// @returns  mass value used for masspoints
    //----------------------------------------------------------------------------------------------------------------------
    float point_mass() const { return m_mass; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns spring constant for structural springs
    /// @returns  spring constant
    //----------------------------------------------------------------------------------------------------------------------
    float spring_constant() const { return m_k; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns damping coefficent
    /// @returns  damping coefficent
    //----------------------------------------------------------------------------------------------------------------------
    float damping() const { return m_damping; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns whether or not we're using damping
    /// @returns  whether damping is on or off
    //----------------------------------------------------------------------------------------------------------------------
    bool useDamping() const { return m_useDamping; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns whether or not collisions are being calculated
    /// @returns  whether collisions are on or off
    //----------------------------------------------------------------------------------------------------------------------
    bool collision() const { return m_collision; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns whether or not every masspoint has been fixed
    /// @returns  whether all masspoints are fixed or not
    //----------------------------------------------------------------------------------------------------------------------
    bool fullClothFixed() const;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns current center of cloth vertices
    /// @returns  center of cloth object
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 center() const;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns vertices of the given triangle
    /// @param[in]  _triNum the number ID of the triangle to which these vertices belong
    /// @returns  vertices of the given triangle
    //----------------------------------------------------------------------------------------------------------------------
    std::vector<ngl::Vec3> vertFromTriNum(const size_t _triNum) const;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief getter returns this object's bounding volume tree
    /// @returns  this cloth's bounding volume tree
    //----------------------------------------------------------------------------------------------------------------------
    BVTree exportTree() { return m_bvtree; }
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief sets whether or not damping is on
    /// @param[in]  _dval turns on or off damping
    //----------------------------------------------------------------------------------------------------------------------
    void setDamping(const bool _dval) { m_useDamping = _dval; }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief sets whether or not collision is on
    /// @param[in]  _cval turns on or off collisions
    //----------------------------------------------------------------------------------------------------------------------
    void setCollision(const bool _cval) { m_collision = _cval; }
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief fixes the four corner points on the cloth
    //----------------------------------------------------------------------------------------------------------------------
    void fixptCorners();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief fixes the top two corner points on the cloth
    //----------------------------------------------------------------------------------------------------------------------
    void fixptHang();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief fixes the side two corner points on the cloth
    //----------------------------------------------------------------------------------------------------------------------
    void fixptFlag();
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief initializes cloth state
    /// @param[in]  _height the height in masspoints of the cloth
    /// @param[in]  _width the width in masspoints of the cloth
    /// @param[in]  _restl the resting length of the main structural springs
    /// @param[in]  _ycoord the y-coordinate this cloth will start at
    /// @param[in]  _vertical whether or not this cloth will be set up on the x-z plane or the x-y plane
    //----------------------------------------------------------------------------------------------------------------------
    void init(size_t _height, size_t _width, float _restl, float _ycoord = 5.0f, bool vertical=false);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief repositions the cloth such that its center is at the x-z origin
    /// @param[in]  _ycoord the height to set the cloth
    //----------------------------------------------------------------------------------------------------------------------
    void reposToOrigin(float _ycoord);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief runs through one step of integrating the cloth over time
    /// @param[in]  _h the time step (in seconds)
    //----------------------------------------------------------------------------------------------------------------------
    void update(const float _h);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief exports triangle information of this cloth, for purposes of drawing and setting up bounding volumes
    /// @param[out]  o_vertexData the triangle vertex data of this cloth
    //----------------------------------------------------------------------------------------------------------------------
    void exportTriangles(std::vector<ngl::Vec3> &o_vertexData);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief detects if this cloth is colliding with another scene element
    /// @param[in]  _eTree the bounding volume tree of another scene element
    /// @returns  the information on triangle collision detections that occured
    //----------------------------------------------------------------------------------------------------------------------
    ColDetectInfo detectCollision(BVTree _eTree) { return m_bvtree.detectCollision(_eTree); }
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief given a triangle number, modifies the vertices of said triangle to the input
    /// if the vertex changed, it will fix the vertex to save collision response time on future updates
    /// @param[in]  _triNum the number of the triangle whose vertices we want to update
    /// @param[in]  _v the updated vertices of the given triangle
    //----------------------------------------------------------------------------------------------------------------------
    void modVertFromTriNum(const size_t _triNum, std::vector<ngl::Vec3> _v);
    //----------------------------------------------------------------------------------------------------------------------

private:
    //----------------------------------------------------------------------------------------------------------------------
    /// @struct Spring "include/Cloth.h"
    /// @brief A simple spring object
    //----------------------------------------------------------------------------------------------------------------------
    struct Spring
    {
        float restl;            //!< resting length of the spring
        ngl::Mat3 jacobian;     //!< values of jacobian matrix, for integration

        //------------------------------------------------------------------------------------------------------------------
        /// @brief user constructor to easily set up spring
        /// @param[in]  _restl resting length of spring
        /// @param[in]  _jacobian holds jacobian matrix belonging to this spring
        //------------------------------------------------------------------------------------------------------------------
        Spring(float _restl, ngl::Mat3 _jacobian) : restl(_restl), jacobian(_jacobian) {;}
    };
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief initializes a spring based on two masspoints and a resting length
    /// @param[in]  _ind0 index of the first masspoint this spring is connected to
    /// @param[in]  _ind1 index of the second masspoint this spring is connected to
    /// @param[in]  _restl resting length of spring
    //----------------------------------------------------------------------------------------------------------------------
    void createSpring(size_t _ind0, size_t _ind1, float _restl);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief updates the values in the each spring's jacobian matrix for integration
    //----------------------------------------------------------------------------------------------------------------------
    void updateSpringJacobians();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief updates the values in the each mass's jacobian matrix for integration
    //----------------------------------------------------------------------------------------------------------------------
    void updateMassJacobians();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief calculates the inverse of the W matrix for integration
    /// @param[out]  o_wiiMatrices the list of W matrices for each masspoint in the cloth
    /// @param[in]  _h the time step (in seconds)
    //----------------------------------------------------------------------------------------------------------------------
    void calcWinverse(std::vector<ngl::Mat3> &o_wiiMatrices, const float _h);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief calculates the Pforce vector for integration
    /// @param[out]  o_pforces the pforce vector for each masspoint in the cloth
    /// @param[in]  _externalf external force vector
    /// @param[in]  _h the time step (in seconds)
    //----------------------------------------------------------------------------------------------------------------------
    void calcPforces(std::vector<ngl::Vec3> &o_pforces, const ngl::Vec3 _externalf, const float _h);
    //----------------------------------------------------------------------------------------------------------------------

    //----------------------------------------------------------------------------------------------------------------------
    /// MEMBER VARIABLES
    //----------------------------------------------------------------------------------------------------------------------
    std::vector<MassPoint> m_mspts;                                         //!< masspoints in this cloth
    std::unordered_map<UnorderedPair, Spring, Hash_UP, Eq_UP> m_springs;    //!< springs in this cloth
    BVTree m_bvtree;                                                        //!< bounding volume tree for the cloth
    size_t m_height = 0;                                                    //!< height of the cloth in masspoints
    size_t m_width = 0;                                                     //!< width of the cloth in masspoints
    float m_mass = 1.0f;                                                    //!< mass of the individual masspoints
    float m_k = 1.0f;                                                       //!< spring constant of the individual springs
    float m_damping = 1.0f;                                                 //!< damping coefficient
    bool m_useDamping = true;                                               //!< whether or not we're using damping
    bool m_collision = false;                                               //!< whether or not collisions are on
};

#endif
