////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

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
static const bool g_Gl2Compatible = false;



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

struct ShaderState {
  GlProgram program;

  // Handles to uniform variables
  GLint h_uLight, h_uLight2;
  GLint h_uProjMatrix;
  GLint h_uModelViewMatrix;
  GLint h_uNormalMatrix;
  GLint h_uColor;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aNormal;

  ShaderState(const char* vsfn, const char* fsfn) {
    readAndCompileShader(program, vsfn, fsfn);

    const GLuint h = program; // short hand

    // Retrieve handles to uniform variables
    h_uLight = safe_glGetUniformLocation(h, "uLight");
    h_uLight2 = safe_glGetUniformLocation(h, "uLight2");
    h_uProjMatrix = safe_glGetUniformLocation(h, "uProjMatrix");
    h_uModelViewMatrix = safe_glGetUniformLocation(h, "uModelViewMatrix");
    h_uNormalMatrix = safe_glGetUniformLocation(h, "uNormalMatrix");
    h_uColor = safe_glGetUniformLocation(h, "uColor");

    // Retrieve handles to vertex attributes
    h_aPosition = safe_glGetAttribLocation(h, "aPosition");
    h_aNormal = safe_glGetAttribLocation(h, "aNormal");

    if (!g_Gl2Compatible)
      glBindFragDataLocation(h, 0, "fragColor");
    checkGlErrors();
  }

};

static const int g_numShaders = 2;
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}
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


// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube1, g_cube2, g_arcball; /*added*/

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
static RigTForm g_objectRbt[2] = {RigTForm(Cvec3(-1,0,0)), RigTForm(Cvec3(1,0,0)) };
//g_objectRbt[0] : cube1, g_objectRbt[1] : cube2
static Cvec3f g_objectColors[2] = {Cvec3f(1, 0, 0), Cvec3f(0,0,1)};
static Cvec3f g_arcballColors = Cvec3f(0, 1, 0);
//g_objectColors[0] : cube1, g_objectColors[1] : cube 2

static int views = 2; //added { 0 : cube 1 , 1: cube 2, 2 : sky }
static int objects = 0; //added { 0 : cube 1 , 1: cube 2, 2 : sky }
static int world = 1; //added { 0 : sky-sky frame 1:world-sky frame }

static RigTForm give_eyeRbt() {
    if (views == 2) {
        return g_skyRbt;
    }
    return g_objectRbt[views];
}

static RigTForm give_objectRbt() {
    if (objects == 2) {
        return g_skyRbt;
    }
    return g_objectRbt[objects];
}

static RigTForm give_auxFrame() {
    if (objects != 2 && views != 2) { //cube i - cube j frame
        return makeMixedFrame(g_objectRbt[objects], g_objectRbt[views]);
    
    }
    if (objects != 2 && views == 2) { //cube i - sky frame
        return makeMixedFrame(g_objectRbt[objects], g_skyRbt);
    }
    if (objects == 2 && views == 2 && world == 1) { //world-sky
        return  linFact(g_skyRbt);
    }
    if (objects == 2 && views == 2 && world == 0) { //sky-sky
        return g_skyRbt;
    }
}

static RigTForm give_arcballRbt() {
    if (objects!=2)  return give_objectRbt(); //cube i-cube j frame
    //world-sky
    return RigTForm();
}

static bool valid_manipulation() {
    if (objects == 2 && views != 2) return false;
    return true;
}

static bool valid_arcball() {
    if ((inv(give_eyeRbt()) * give_arcballRbt()).getTranslation()[2] > -CS175_EPS) return false; // Don't use arcball when it isn't visible
    if (objects != 2 && objects != views) return true; //manipulating cube, not ego
    if (objects == 2 && views == 2 & world == 1) return true; //wolrd-sky view
    return false;
}

static RigTForm eyeRbt = give_eyeRbt(); // 
static RigTForm objectRbt = give_objectRbt();
static RigTForm auxFrame = give_auxFrame();
static RigTForm g_arcballRbt = give_arcballRbt();

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
  g_cube1.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));

  /*added*/
  int ibLen1, vbLen1;
  getCubeVbIbLen(vbLen1, ibLen1); //24, 36

  // Temporary storage for cube geometry
  vector<VertexPN> vtx1(vbLen1);
  vector<unsigned short> idx1(ibLen1);

  makeCube(1, vtx1.begin(), idx1.begin());
  g_cube2.reset(new Geometry(&vtx1[0], &idx1[0], vbLen1, ibLen1));
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

// takes MVM and its normal matrix to the shaders
static void sendModelViewNormalMatrix(const ShaderState& curSS, const Matrix4& MVM, const Matrix4& NMVM) {
  GLfloat glmatrix[16];
  MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
  safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

  NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
  safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
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

static void drawStuff() {
  // short hand for current shader state
  const ShaderState& curSS = *g_shaderStates[g_activeShader];

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

  // draw ground
  // ===========
  //
  const RigTForm groundRbt = RigTForm();  // identity
  Matrix4 MVM = rigTFormToMatrix(invEyeRbt * groundRbt);
  Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
  g_ground->draw(curSS);

  // draw cubes
  // ==========
  MVM = rigTFormToMatrix( invEyeRbt * g_objectRbt[0]);
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_objectColors[0][0], g_objectColors[0][1], g_objectColors[0][2]);
  g_cube1->draw(curSS);
  /*added*/
  MVM = rigTFormToMatrix(invEyeRbt * g_objectRbt[1]);
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_objectColors[1][0], g_objectColors[1][1], g_objectColors[1][2]);
  g_cube2->draw(curSS);

  //draw arcball
  if (valid_arcball()) { //valid?

      g_arcballRbt = give_arcballRbt();
      if(!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))){ //do not update when the user is translating in the Z direction
          g_arcballScale = getScreenToEyeScale((invEyeRbt * g_arcballRbt).getTranslation()[2], g_frustFovY, g_windowHeight);
      }
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //draw wire frame

      Matrix4 scale_matrix = Matrix4::makeScale(g_arcballScale * g_arcballScreenRadius);
      MVM = rigTFormToMatrix(invEyeRbt * g_arcballRbt) * scale_matrix;
      NMVM = normalMatrix(MVM);
      sendModelViewNormalMatrix(curSS, MVM, NMVM);
      safe_glUniform3f(curSS.h_uColor, g_arcballColors[0], g_arcballColors[1],g_arcballColors[2]);
      g_arcball->draw(curSS);
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }

  
}

static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff();

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
    return std::sqrt(max(0.0,pow(g_arcballScreenRadius, 2) - pow(x - x_arcball, 2) - pow(y - y_arcball, 2)));
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
          eyeRbt = give_eyeRbt(); //eyeRbt
          objectRbt = give_objectRbt(); //objectRbt

          if (objects != 2) {// manipulating one of the cubes 
              if (objects != views) g_objectRbt[objects] = doMtoOwrtA(auxFrame, m, objectRbt); //arcball
              else g_objectRbt[objects] = doMtoOwrtA(eyeRbt, transFact(m)*inv(linFact(m)), eyeRbt); //perform ego motion (else at the document, only invert the sign of the rotations)
          }
          if (objects == 2 && world == 1) { //world sky view //arcball
              g_skyRbt = doMtoOwrtA(auxFrame, inv(m), objectRbt);
          }
          if (objects == 2 && world == 0) {//sky sky view //ego motion (else at the document, only invert the sign of the rotations)
              g_skyRbt = doMtoOwrtA(eyeRbt, transFact(m) * inv(linFact(m)), eyeRbt);
          }
          glutPostRedisplay(); // we always redraw if we changed the scene
      }
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
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
    case 'o':
        if (objects == 1) {
            cout << "current object is sky" << endl;
            objects = 2;
        }
        else if (objects == 2) {
            cout << "current object is cube1" << endl;
            objects = 0;
        }
        else {
            cout << "current object is cube2" << endl;
            objects++;
        }
            
        break;
    case 'm':
        if (world == 1 && objects ==2 && views==2) {
            cout << "w.r.t. sky-sky frame" << endl;
            world = 0;
        }
        else if(world == 0 && objects == 2 && views == 2) {
            cout << "w.r.t. world-sky frame" << endl;
            world++;
        }
        break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 2");                       // title the window

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

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
