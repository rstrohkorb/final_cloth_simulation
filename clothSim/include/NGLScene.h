#ifndef NGLSCENE_H_
#define NGLSCENE_H_
#include <memory>
#include <ngl/Vec3.h>
#include <ngl/Vec4.h>
#include <ngl/Mat4.h>
#include <ngl/AbstractVAO.h>
#include "Cloth.h"
#include "SphereObj.h"
#include "BVTree.h"
#include "WindowParams.h"
// this must be included after NGL includes else we get a clash with gl libs
#include <QOpenGLWindow>
//----------------------------------------------------------------------------------------------------------------------
/// @file NGLScene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class NGLScene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
//----------------------------------------------------------------------------------------------------------------------

class NGLScene : public QOpenGLWindow
{
  public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief ctor for our NGL drawing class
    /// @param [in] parent the parent window to the class
    //----------------------------------------------------------------------------------------------------------------------
    NGLScene();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief dtor must close down ngl and release OpenGL resources
    //----------------------------------------------------------------------------------------------------------------------
    ~NGLScene() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief events that run every clock cycle once timer starts
    //----------------------------------------------------------------------------------------------------------------------
    void timerEvent(QTimerEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the initialize class is called once when the window is created and we have a valid GL context
    /// use this to setup any default GL stuff
    //----------------------------------------------------------------------------------------------------------------------
    void initializeGL() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we want to draw the scene
    //----------------------------------------------------------------------------------------------------------------------
    void paintGL() override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we resize the window
    //----------------------------------------------------------------------------------------------------------------------
    void resizeGL(int _w, int _h) override;

private:

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Qt Event called when a key is pressed
    /// @param [in] _event the Qt event to query for size etc
    //----------------------------------------------------------------------------------------------------------------------
    void keyPressEvent(QKeyEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called every time a mouse is moved
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseMoveEvent (QMouseEvent * _event ) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is pressed
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mousePressEvent ( QMouseEvent *_event) override;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse button is released
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void mouseReleaseEvent ( QMouseEvent *_event ) override;

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this method is called everytime the mouse wheel is moved
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    //----------------------------------------------------------------------------------------------------------------------
    void wheelEvent( QWheelEvent *_event) override;
    /// @brief windows parameters for mouse control etc.
    WinParams m_win;
    /// position for our model
    ngl::Vec3 m_modelPos;

    /// cloth object
    Cloth m_cloth;
    /// sphere object
    SphereObj m_sphere;
    /// whether or not we'll be turning on the sphere
    bool m_sphereOn;
    /// whether or not we'll be doing collision
    bool m_collisionOn=false;
    bool m_wireframeOn=false;
    int m_timerId;
    /// view matrix
    ngl::Mat4 m_view;
    /// project matrix
    ngl::Mat4 m_project;
    /// VAO for the cloth triangles
    std::unique_ptr<ngl::AbstractVAO> m_clothVAO;
    /// VAO for the sphere triangle strips
    std::unique_ptr<ngl::AbstractVAO> m_sphereVAO;
    /// load matrix to shader
    void loadMatrixToShader(const ngl::Mat4 &_tx, const ngl::Vec4 &_color);
    void loadMatrixToCheckerShader(const ngl::Mat4 &_tx);
    /// Respond to detected collisions
    void collisionResponse(ColDetectInfo _cdi);
    void collisionResponseFail(ColDetectInfo _cdi);
    ngl::Vec3 edgeFromNum(const size_t _num, std::vector<ngl::Vec3> _vert);

    bool m_writeOutCloth = false;
    std::string m_filenameDefault = "results/hangTest/msClothHang.";

};



#endif
