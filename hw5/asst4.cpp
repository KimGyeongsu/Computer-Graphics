////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////
#include <list>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <fstream>

#include <GL/glew.h>
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

#include "quat.h"
#include "rigtform.h"
#include "arcball.h"

#include "asstcommon.h"
#include "drawer.h"
#include "picker.h"
#include "scenegraph.h"

#include "sgutils.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;



static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static double g_arcballScreenRadius = 1.0;
static double g_arcballScale = 1.0;

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3; // 3 shaders instead of 2
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
  {}

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex& v) {
    *this = v;
  }

  VertexPN& operator = (const GenericVertex& v) {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry {
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState& curSS) {
    // Enable the attributes used by our shader
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

    // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);
  }
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_arcball; /*added*/
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static Cvec3f g_arcballColors = Cvec3f(0, 1, 0);


static int views = 2; //added { 0 : cube 1 , 1: cube 2, 2 : sky }
static int world = 1; //added { 0 : sky-sky frame 1:world-sky frame }
static int pick_mode = 0; //added {1 : picking}
static int pick_well = 0; //{1 : picked well)
static int animating = 0;

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes(initialized)
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback


static shared_ptr<SgRbtNode> give_eyeRbtNode() {
    if (views == 0) return g_robot1Node;
    if (views == 1) return g_robot2Node;
    if (views == 2) return g_skyNode;
}

static RigTForm give_eyeRbt() {
    return getPathAccumRbt(g_world, give_eyeRbtNode());
}

static RigTForm give_eyeRbt_only() {
    return give_eyeRbtNode()->getRbt();
}


static shared_ptr<SgRbtNode> give_objRbtNode() {
    return g_currentPickedRbtNode;
}

static RigTForm give_objRbt() {
    return getPathAccumRbt(g_world, give_objRbtNode());
}

static RigTForm give_objRbt_only() {
    return give_objRbtNode()->getRbt();
}

static RigTForm give_auxFrame() {
    if (g_currentPickedRbtNode != g_skyNode ) { //else
        return inv(getPathAccumRbt(g_world, give_objRbtNode(), 1)) * makeMixedFrame(give_objRbt(), give_eyeRbt());
    }
    if (g_currentPickedRbtNode == g_skyNode && views == 2 && world == 1) { //world-sky
        return inv(getPathAccumRbt(g_world, give_objRbtNode(), 1))* linFact(getPathAccumRbt(g_world, g_skyNode));
    }
    if (g_currentPickedRbtNode == g_skyNode && views == 2 && world == 0) { //sky-sky
        return inv(getPathAccumRbt(g_world, give_objRbtNode(), 1)) * getPathAccumRbt(g_world, g_skyNode);
    }
}

static RigTForm give_arcballRbt() {
    if (give_objRbtNode() != g_skyNode)  return give_objRbt();
    //world-sky
    return RigTForm();
}

static bool valid_manipulation() {
    if (give_objRbtNode() == g_skyNode && views != 2) return false;
    return true;
}

static bool valid_arcball() {
    if ((inv(give_eyeRbt()) * give_arcballRbt()).getTranslation()[2] > -CS175_EPS) return false; // Don't use arcball when it isn't visible
    if (give_objRbtNode() != g_skyNode && give_objRbtNode() != give_eyeRbtNode()) return true; //manipulating cube, not ego
    if (give_objRbtNode() == g_skyNode && views == 2 & world == 1) return true; //wolrd-sky view
    return false;
}

static RigTForm eyeRbt; 
static RigTForm objectRbt; 
static RigTForm auxFrame;
static RigTForm g_arcballRbt;


//////////////////////////////////////////////////////////////////////////////
static list<vector<RigTForm>> keyframes;
static list<vector<RigTForm>>::iterator cur_iter = keyframes.end();
static int frame_number = -1;
static int numRbtNodes = 22;


static void copy_curFrame_to_Scene();
static void copy_Scene_to_curFrame();
static void create_newFrame_set_as_curFrame();
static void create_newFrame_set_as_curFrame_when_empty();
static void delete_curFrame();
static void write_file(const char* filename);
static void read_file(const char* filename);
static void animateTimerCallback(int ms);
///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen); //24, 36

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));

}

static void initArcball() {
    int ibLen, vbLen;
    getSphereVbIbLen(15, 15, vbLen, ibLen);
    vector<VertexPN> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(1, 15, 15, vtx.begin(), idx.begin());
    g_arcball.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}


// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static void drawStuff(const ShaderState& curSS, bool picking) {

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the skyRbt as the eyeRbt
  //const Matrix4 eyeRbt = g_skyRbt;
  eyeRbt = give_eyeRbt();
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  if (!picking) {
      Drawer drawer(invEyeRbt, curSS);
      g_world->accept(drawer);

      // draw arcball as part of asst3
      if (valid_arcball()) { //valid?
          g_arcballRbt = give_arcballRbt();
          if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))) { //do not update when the user is translating in the Z direction
              g_arcballScale = getScreenToEyeScale((invEyeRbt * g_arcballRbt).getTranslation()[2], g_frustFovY, g_windowHeight);
          }
          glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //draw wire frame

          Matrix4 scale_matrix = Matrix4::makeScale(g_arcballScale * g_arcballScreenRadius);
          Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_arcballRbt) * scale_matrix;
          Matrix4 NMVM = normalMatrix(MVM);
          sendModelViewNormalMatrix(curSS, MVM, NMVM);
          safe_glUniform3f(curSS.h_uColor, g_arcballColors[0], g_arcballColors[1], g_arcballColors[2]);
          g_arcball->draw(curSS);
          glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
  }
  else {
      Picker picker(invEyeRbt, curSS);
      g_world->accept(picker);
      glFlush();
      g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
      if (g_currentPickedRbtNode == g_groundNode || g_currentPickedRbtNode == NULL) {
          g_currentPickedRbtNode = give_eyeRbtNode();   // eye's transform node is activated
          pick_well = 0;
      }
      else pick_well = 1;

      
  }

  
}

static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  if(pick_mode==1 && !(g_mouseLClickButton && !g_mouseRClickButton)) drawStuff(*g_shaderStates[g_activeShader], false); //not yet picked
  else drawStuff(*g_shaderStates[g_activeShader], pick_mode);

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static double give_z_coordinate(const double x, const double y, const double x_arcball, const double y_arcball) {
    return sqrt(max(0.0,pow(g_arcballScreenRadius, 2) - pow(x - x_arcball, 2) - pow(y - y_arcball, 2)));
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  RigTForm m = RigTForm();


  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) && valid_arcball()) {
      g_arcballScale = getScreenToEyeScale((inv(give_eyeRbt()) * give_arcballRbt()).getTranslation()[2], g_frustFovY, g_windowHeight);
  }

  if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
    if (valid_arcball()) { //arcball rotation
        Cvec3 center_arcball = Cvec3(Cvec2(getScreenSpaceCoord((inv(give_eyeRbt()) * give_arcballRbt()).getTranslation(), makeProjectionMatrix(),  g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight)),0.0);
        Cvec3 before = Cvec3(g_mouseClickX, g_mouseClickY, give_z_coordinate(g_mouseClickX, g_mouseClickY, center_arcball[0], center_arcball[1]));
        Cvec3 after = Cvec3(x, g_windowHeight - y - 1, give_z_coordinate(x, g_windowHeight - y - 1, center_arcball[0], center_arcball[1]));

        Cvec3 v1 = normalize(before - center_arcball);
        Cvec3 v2 = normalize(after - center_arcball);
        m.setRotation(Quat(0, v2) * inv(Quat(0, v1)));
    }
    else  m.setRotation(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
    if(valid_arcball()) m.setTranslation(Cvec3(dx, dy, 0) * g_arcballScale);
    else m.setTranslation(Cvec3(dx, dy, 0) * 0.01);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
    if (valid_arcball()) m.setTranslation(Cvec3(0, 0, -dy) * g_arcballScale);
    else m.setTranslation(Cvec3(0, 0, -dy) * 0.01);
  }

  RigTForm auxFrame = give_auxFrame();
  if (g_mouseClickDown) {
      if (valid_manipulation()) { //if it is valid manipulation

          if (give_objRbtNode()!= g_skyNode) {// manipulating one of the cubes 
              if (give_objRbtNode() != give_eyeRbtNode()) give_objRbtNode()->setRbt(doMtoOwrtA(auxFrame, m, give_objRbt_only())); //arcball
              else give_objRbtNode()->setRbt(doMtoOwrtA(auxFrame, transFact(m)*inv(linFact(m)), give_objRbt_only())); //perform ego motion (else at the document, only invert the sign of the rotations)
          }
          if (give_objRbtNode() == g_skyNode && world == 1) { //world sky view //arcball
              g_skyNode->setRbt(doMtoOwrtA(auxFrame, inv(m), give_objRbt_only()));
          }
          if (give_objRbtNode() == g_skyNode && world == 0) {//sky sky view //ego motion (else at the document, only invert the sign of the rotations)
              g_skyNode->setRbt(doMtoOwrtA(auxFrame, transFact(m) * inv(linFact(m)), give_objRbt_only()));
          }
          glutPostRedisplay(); // we always redraw if we changed the scene
      }
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}

static void pick() { 
    // We need to set the clear color to black, for pick rendering.
    // so let's save the clear color
    GLdouble clearColor[4];
    glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

    glClearColor(0, 0, 0, 0);

    // using PICKING_SHADER as the shader
    glUseProgram(g_shaderStates[PICKING_SHADER]->program);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawStuff(*g_shaderStates[PICKING_SHADER], true);

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
    //glutSwapBuffers();

    //Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();
}


static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

  if (pick_mode==1 && g_mouseLClickButton && !g_mouseRClickButton) {
      pick();
      pick_mode = 0;
      if (pick_well == 0) cout << "No part picked" << endl;
      else cout << "Part picked" << endl;
      cout << "Picking mode is off" << endl;
  }

  glutPostRedisplay();
}


static void keyboard(const unsigned char key, const int x, const int y) {
    //added
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "o\t\tCycle object to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
    //added
    case 'v':
        if (views == 1) {
            cout << "current view is sky" << endl;
        }
        if (views == 2) {
            cout << "current view is cube1" << endl;
        }
        if (views == 0) {
            cout << "current view is cube2" << endl;
        }
        if (views == 2) views = 0;
        else views++;
        break;
    case 'm':
        if (world == 1 && give_objRbtNode() == g_skyNode && views==2) {
            cout << "w.r.t. sky-sky frame" << endl;
            world = 0;
        }
        else if(world == 0 && give_objRbtNode() == g_skyNode && views == 2) {
            cout << "w.r.t. world-sky frame" << endl;
            world++;
        }
        break;
    case 'p': //eye's transform node is activated, once node is activated : 
        //for debugging purpose, just print out the node infor
        pick_mode = 1;
        cout << "Picking mode is on" << endl;
        break;
    case 32: //space key
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (!keyframes.empty()) copy_curFrame_to_Scene();
        else cout << "No key frame defined" << endl;
        break;
    case 'u':
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (!keyframes.empty()) {
            copy_Scene_to_curFrame();
            break;
        }
    case 'n' :
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (!keyframes.empty()) create_newFrame_set_as_curFrame();
        else create_newFrame_set_as_curFrame_when_empty();
        break;
    case 62 :
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (keyframes.empty() || frame_number == keyframes.size()-1) { //++cur_iter == keyframes.end()
            cout << "cannot move to the next frame" << endl;
        }
        else {
            frame_number++;
            cur_iter++;
            copy_curFrame_to_Scene();
        }
        break;
    case 60 :
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (keyframes.empty() || frame_number == 0) {
            cout << "cannot move to the front frame" << endl;
        }
        else {
            frame_number--;
            cur_iter--;
            copy_curFrame_to_Scene();
        }
        break;
    case 'd' :
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        if (keyframes.empty()) {
            cout << "cannot delete the frame. keframes are empty" << endl;
        }
        else delete_curFrame();
        break;
    case 'w':
        write_file("animation.txt");
        break;
    case 'i':
        if (animating == 1) {
            cout << "cannot operate when playing animation" << endl;
            break;
        }
        read_file("animation.txt");
        break;
    case 'y': //animation 중일 때 조작 안되게 하는거 추가
        if (keyframes.size() < 4) cout << "Cannot play animation with less than 4 keyframes." << endl;
        else if (animating == 0) {
            animating = 1;
            cout << "Playing animation..." << endl;
            animateTimerCallback(0);
        }
        else {
            animating = 0;
            vector<shared_ptr<SgRbtNode>> rbtNodes;
            dumpSgRbtNodes(g_world, rbtNodes);

            cur_iter = keyframes.end();
            cur_iter--;
            cur_iter--;
            frame_number = keyframes.size() - 2;
            for (int i = 0; i < rbtNodes.size(); i++) {
                rbtNodes[i]->setRbt((*cur_iter)[i]);
            }
            cout << "Stopping animation..." << endl;
        }
        break;
    case '+':
        if (g_msBetweenKeyFrames != 100) g_msBetweenKeyFrames -= 100;
        cout << g_msBetweenKeyFrames;
        cout << " ms between keyframes." << endl;
        break;;
    case '-':
        if (g_msBetweenKeyFrames != 10000) g_msBetweenKeyFrames += 100;
        cout << g_msBetweenKeyFrames; 
        cout<<" ms between keyframes." << endl;
        break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 5");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initCubes();
  initArcball();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {
    /*head, left/right upper arm, left/right lower arm, left/right upper leg, left/right lower leg.
    * dot product
    */
    const double ARM_LEN = 0.7,
        ARM_THICK = 0.25,
        TORSO_LEN = 1.5,
        TORSO_THICK = 0.25,
        TORSO_WIDTH = 1,
        HEAD_SIZE = 0.25;
    const int NUM_JOINTS = 10,
        NUM_SHAPES = 10;

    struct JointDesc {
        int parent;
        float x, y, z;
    };

    JointDesc jointDesc[NUM_JOINTS] = {
      {-1}, // torso
      {0,  TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper right arm
      {1,  ARM_LEN, 0, 0}, // lower right arm
      {0,  -TORSO_WIDTH / 2, TORSO_LEN / 2, 0},//upper left arm
      {3,  -ARM_LEN, 0, 0},//lower left arm
      {0, TORSO_WIDTH / 2 - ARM_THICK / 2, -TORSO_LEN / 2, 0},//upper right leg
      {5, 0, -ARM_LEN, 0},//lower right leg
      {0, -TORSO_WIDTH / 2 + ARM_THICK / 2, -TORSO_LEN / 2, 0},//upper left leg
      {7, 0, -ARM_LEN, 0},//lower left leg
      {0, 0,TORSO_LEN / 2, 0} //head
    };

    struct ShapeDesc {
        int parentJointId;
        float x, y, z, sx, sy, sz;
        shared_ptr<Geometry> geometry;
    };

    ShapeDesc shapeDesc[NUM_SHAPES] = {
      {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
      {1, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
      {2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
      //upper left arm
      {3, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube},
      //lower left arm
      {4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube},
      //upper right leg
      {5, 0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube},
      //lower right leg
      {6, 0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube},
      //upper left leg
      {7, 0, -ARM_LEN / 2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube},
      //lower left leg
      {8, 0, -ARM_LEN / 2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube},
      //head
      {9, 0, HEAD_SIZE, 0, HEAD_SIZE, HEAD_SIZE, HEAD_SIZE, g_arcball}
    };

    shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (jointDesc[i].parent == -1)
            jointNodes[i] = base;
        else {
            jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
            jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
        }
    }
    for (int i = 0; i < NUM_SHAPES; ++i) {
        shared_ptr<MyShapeNode> shape(
            new MyShapeNode(shapeDesc[i].geometry,
                color,
                Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                Cvec3(0, 0, 0),
                Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
        jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
    }
}

static void initScene() {
    g_world.reset(new SgRootNode());

    g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

    g_groundNode.reset(new SgRbtNode());
    g_groundNode->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_currentPickedRbtNode = g_robot1Node;
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
    constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);


    //initialize the global variable
    eyeRbt = give_eyeRbt(); 
    objectRbt = getPathAccumRbt(g_world, g_robot1Node);
    auxFrame = inv(getPathAccumRbt(g_world, g_robot1Node, 1)) * makeMixedFrame(objectRbt, eyeRbt);
    g_arcballRbt = objectRbt;
}



int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();
    initScene();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}

static void copy_curFrame_to_Scene() {
    vector<shared_ptr<SgRbtNode>> rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);
    for (int i = 0; i < rbtNodes.size(); i++) {
        rbtNodes[i]->setRbt((*cur_iter)[i]);
    }
    cout << "Loading current key frame [";
    cout << frame_number;
    cout << "] to scene graph" << endl;
}
static void copy_Scene_to_curFrame() {
    vector<shared_ptr<SgRbtNode>> rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);
    for (int i = 0; i < rbtNodes.size(); i++) {
        (*cur_iter)[i] = rbtNodes[i]->getRbt();
    }
    cout << "Copying scene graph to current frame [";
    cout << frame_number;
    cout << "]" << endl;
}

static void create_newFrame_set_as_curFrame() {
    vector<RigTForm> temp((*cur_iter).size(), RigTForm());
    keyframes.insert(++cur_iter, temp);
    --cur_iter;
    cout << "Create new frame[";
    cout << ++frame_number;
    cout << "]." << endl;
    copy_Scene_to_curFrame();
}

static void create_newFrame_set_as_curFrame_when_empty() {
    vector<shared_ptr<SgRbtNode>> rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);

    vector<RigTForm> temp(rbtNodes.size(), RigTForm());
    for (int i = 0; i < temp.size(); i++) {
        temp[i] = rbtNodes[i]->getRbt();
    }
    keyframes.insert(cur_iter, temp);
    --cur_iter;
    ++frame_number;
    cout << "Create new frame[0].\nCopying scene graph to current frame[0]" << endl;
}

static void delete_curFrame() {
    list<vector<RigTForm>>::iterator temp_iter;

    if (keyframes.size() == 1) {
        frame_number--;
        keyframes.erase(cur_iter);
        cur_iter = keyframes.end();
        cout << "delete current frame[0]" << endl;
    }

    else if (frame_number == 0) {
        temp_iter = cur_iter;
        ++temp_iter;
        keyframes.erase(cur_iter);
        cur_iter = temp_iter;
        cout << "delete current frame[";
        cout << frame_number;
        cout << "]" << endl;


        copy_curFrame_to_Scene();
    }

    else {
        temp_iter = cur_iter;
        --temp_iter;
        keyframes.erase(cur_iter);
        cur_iter = temp_iter;

        cout << "delete current frame[";
        cout << frame_number;
        cout << "]" << endl;

        frame_number--;
        copy_curFrame_to_Scene();
    }
}

static void write_file(const char *filename) {
    ofstream f(filename, ios::binary);
    f << keyframes.size() << ' ' << numRbtNodes << '\n';

    for (list<vector<RigTForm>>::iterator iter = keyframes.begin(), end = keyframes.end(); iter != end; ++iter) {
        for (int j = 0; j < numRbtNodes; j++) {
            Quat r = (*iter)[j].getRotation();
            Cvec3 t = (*iter)[j].getTranslation();
            f << r[0] << ' ' << r[1] << ' ' << r[2] << ' ' << r[3] << ' ' << t[0] << ' ' << t[1] << ' ' << t[2] << '\n';
        }
    }
    f.close();
    cout << "Writing animation to ";
    cout << filename << endl;
}

static void read_file(const char* filename) {
    int numFrames, numRbtsPerFrame;

    ifstream f(filename, ios::binary);
    string line;

    if (f.is_open()) {
        getline(f, line);
        numFrames = stoi(line.substr(0, line.find(" ")));
        numRbtsPerFrame = stoi(line.erase(0, line.find(" ") + 1));
    }

    keyframes.clear();
    cur_iter = keyframes.end();
    frame_number = -1;
    numRbtNodes = numRbtsPerFrame;

    if (numFrames == 0) { // if 0 frames are exits
        cout << "Reading animation from ";
        cout << filename << endl;
        cout << "0 frames read." << endl;
    }

    else {
        for (int k = 0; k < numFrames; k++) {
            vector<RigTForm> RBTs;
            for (int l = 0; l < numRbtsPerFrame; l++) {
                getline(f, line);
                int pos = 0;
                Quat r = Quat();
                Cvec3 t = Cvec3();

                for (int i = 0; i < 4; i++) {
                    pos = line.find(" ");
                    r[i] = stod(line.substr(0, pos));
                    line.erase(0, pos + 1);
                }
                for (int i = 0; i < 3; i++) {
                    pos = line.find(" ");
                    t[i] = stod(line.substr(0, pos));
                    line.erase(0, pos + 1);
                }
                RBTs.push_back(RigTForm(t, r));
            }
            keyframes.insert(cur_iter, RBTs);
        }
        frame_number = 0;
        cur_iter = keyframes.begin();

        cout << "Reading animation from ";
        cout << filename << endl;
        cout << numFrames;
        cout << " frames read." << endl;

        copy_curFrame_to_Scene();
    }
    f.close();
}
// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if we are at the end of the animation
// sequence, or false otherwise.

static void iterator_move(int n) {
    if (n > 0) for (int i = 0; i < n; i++) cur_iter++;
    else for (int i = 0; i < abs(n); i++) cur_iter--;
}
static bool interpolateAndDisplay(float t) {
    vector<shared_ptr<SgRbtNode>> rbtNodes;
    dumpSgRbtNodes(g_world, rbtNodes);
    if (animating == 0) return false;


    if (t >= keyframes.size() - 3) {

        cur_iter = keyframes.end();
        iterator_move(-2);
        frame_number = keyframes.size() - 2;
        for (int i = 0; i < rbtNodes.size(); i++) {
            rbtNodes[i]->setRbt((*cur_iter)[i]);
        }
        glutPostRedisplay();
        return true;
    }
    else {

        cur_iter = keyframes.begin();
        iterator_move(1 + (int)t);
        vector<RigTForm> prev = (*cur_iter);
        vector<RigTForm> next = (*(++cur_iter));
        vector<RigTForm> interpolation(prev.size(), RigTForm());

        for (int i = 0; i < prev.size(); i++) {

            rbtNodes[i] ->setRbt(interpolate(prev[i], next[i], t - (int)t));
        }

        glutPostRedisplay();
        return false;
    }
}
// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
    float t = (float)ms / (float)g_msBetweenKeyFrames;
    bool endReached = interpolateAndDisplay(t);
    if (!endReached && animating==1) {
        glutTimerFunc(1000 / g_animateFramesPerSecond, animateTimerCallback, ms + 1000 / g_animateFramesPerSecond);
    }
    else  {
        animating = 0;
        vector<shared_ptr<SgRbtNode>> rbtNodes;
        dumpSgRbtNodes(g_world, rbtNodes);
        cur_iter = keyframes.end();
        iterator_move(-2);
        frame_number = keyframes.size() - 2;
        for (int i = 0; i < rbtNodes.size(); i++) {
            rbtNodes[i]->setRbt((*cur_iter)[i]);
        }
        glutPostRedisplay();
        cout << "Finished playing animation\nNow at frame [";
        cout << frame_number;
        cout << "]" << endl;
    }
}
