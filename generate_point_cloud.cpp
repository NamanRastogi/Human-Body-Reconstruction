/*
Author:  Naman Rastogi
    M.Tech., CSE
    IIT Bombay
*/

#define ASSERT(condition, message)												\
    if (! (condition)) {														\
        std::cerr << "Assertion `" #condition "` failed in " << __FILE__		\
                  << " line " << __LINE__ << ": " << message << std::endl;		\
        std::terminate();														\
    }

#define ERROR(message)															\
{																				\
    std::cerr << "ERROR [" << __FUNCTION__ << "] : " << message << std::endl;	\
    std::terminate(); 															\
}


#define WARNING(message)														\
    std::cerr << "WARNING [" << __FUNCTION__ << "] : " << message << std::endl;



/*
// =====================================================================
//
// HEADER FILES
//
// =====================================================================
*/

// FL and OpenGL Headers
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Color_Chooser.H>
#include <FL/glut.H>
#include <FL/gl.h>
#include <GL/glu.h>

// Basic C++ Headers
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility> // std::pair
#include <random>
#include <set>
#include <functional> // std::bind

// GLM Headers
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// ASSIMP Headers
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>


// Namespace
using namespace std;


/*
 **********************************************************************
 **********************************************************************
 *
 * FUNCTION SIGNATURES
 *
 **********************************************************************
 **********************************************************************
*/

// GLUT Callbacks
void displayCallback();
void reshapeCallback( int new_width, int new_height );
void keyboardCallback( unsigned char key, int x, int y );
void keyboardSpecialDownCallback( int key, int x, int y );
void mouseButtonCallback( int button, int state, int x, int y );
void mouseMotionCallback( int x, int y );

// Other Callbacks
void loadScene_OBJ( Fl_Widget* w, void* data );
void loadScene_PLY( Fl_Widget* w, void* data );
void loadPointCloud_OBJ( Fl_Widget* w, void* data );
void loadPointCloud_PTCLD( Fl_Widget* w, void* data );
void screenshot( Fl_Widget* w, void* data );
void subset_pt_cld( Fl_Widget* w, void* data );
void editSceneParam( Fl_Widget* w, void* data );



/*
 **********************************************************************
 **********************************************************************
 *
 * GLOBAL VARIABLES
 *
 **********************************************************************
 **********************************************************************
*/

// Window Parameters
int WIDTH = 640, HEIGHT = 480;

// OpenGL Camera Parameters
glm::vec3 cameraPosition;
glm::vec3 cameraDirection, cameraRightDirection;
glm::vec3 upDirection = glm::vec3( 0.0f, 1.0f, 0.0f );
float cameraMoveFront = 0.0f, cameraMoveRight = 0.0f, cameraMoveUp = 0.0f;

double yaw = -90.0, pitch = 0.0;
int lastMousePosX, lastMousePosY;
float deltaYaw, deltaPitch;

float cameraSpeed = 0.5f;
float mouseSensetivity = 0.2f;

// OpenGL Scene to be rendered
glm::vec3 BB_pCorner, BB_nCorner;

vector<glm::vec3> pointCloud;

struct object {
    char sceneType;
    vector <glm::vec3> Faces;
    vector <glm::vec3> Vertices;
    vector <glm::vec3> Normals;
    glm::vec3 pCorner, nCorner;
}* obj;
int noOfMeshes;
bool normals_available = false;

char sceneType = 0;


// Light and Material Parameters
glm::vec3 lightDirection = glm::normalize( glm::vec3( 1.0f, 1.0f, 1.0f ) );
glm::vec3 lightColor = glm::vec3( 1.0f, 1.0f, 1.0f );
glm::vec3 objectColorDiffuse = glm::vec3( 1.0f, 1.0f, 1.0f );
glm::vec3 objectColorAmbient = objectColorDiffuse * glm::vec3( 0.1f, 0.1f, 0.1f );




/*
 **********************************************************************
 **********************************************************************
 *
 * FUNCTION DEFINITIONS
 *
 **********************************************************************
 **********************************************************************
*/

int main( int argc, char* argv[] )
{
    // Define FLTK Window
    Fl_Window win( WIDTH + 10, HEIGHT + 10 + 25, "Object Viewer" );

    // Add Menu Bar on FLTK widow
    Fl_Menu_Bar* menu = new Fl_Menu_Bar( 0, 0, WIDTH + 10, 25 );
    menu -> add( "Open/Mesh: Wavefront File (OBJ) with normals", 0, loadScene_OBJ );
    menu -> add( "Open/Mesh: Stanford File (PLY)", 0, loadScene_PLY );
    menu -> add( "Open/Point-Cloud: (OBJ)", 0, loadPointCloud_OBJ );
    menu -> add( "Open/Point-Cloud: (PTCLD)", 0, loadPointCloud_PTCLD );
    menu -> add( "Save/Depth Image (PGM)", 0, screenshot, (void*)"d" );
    menu -> add( "Save/RGB Image (PPM)", 0, screenshot, (void*)"c" );
    menu -> add( "Save/Point Cloud from current view (OBJ)", 0, screenshot, (void*)"p" );
    menu -> add( "Save/Point Cloud (subset) of current mesh (PTCLD)", 0, subset_pt_cld );
    menu -> add( "Edit/Light Direction", 0, editSceneParam, (void*)"d" );
    menu -> add( "Edit/Light Color", 0, editSceneParam, (void*)"c" );
    menu -> add( "Edit/Object Color", 0, editSceneParam, (void*)"o" );

    win.resizable( win );
    win.show( argc, argv );

    win.begin();
    glutInitWindowSize( WIDTH, HEIGHT );
    glutInitWindowPosition( 5, 25 + 5 );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE );
    glutCreateWindow( "" );
    glEnable( GLUT_MULTISAMPLE );
    glEnable( GL_DEPTH_TEST );

    glutDisplayFunc( displayCallback );
    glutIdleFunc( displayCallback );
    glutReshapeFunc( reshapeCallback );
    glutKeyboardFunc( keyboardCallback );
    glutSpecialFunc( keyboardSpecialDownCallback );
    glutMouseFunc( mouseButtonCallback );
    glutMotionFunc( mouseMotionCallback );
    win.end();

    glutMainLoop();

    return 0;
}


void displayCallback()
{
    glMatrixMode( GL_MODELVIEW );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );

    glLoadIdentity();

    if( pitch > 89.0f )
        pitch = 89.0f;
    if( pitch < -89.0f )
        pitch = -89.0f;
    if( ( pitch + deltaPitch ) > 89.0f )
        deltaPitch = 89.0f - pitch;
    if( ( pitch + deltaPitch ) < -89.0f )
        deltaPitch = -89.0f - pitch;

    cameraDirection = glm::normalize( glm::vec3(
        glm::cos( glm::radians(pitch + deltaPitch) ) * glm::cos( glm::radians(yaw + deltaYaw) ),
        glm::sin( glm::radians(pitch + deltaPitch) ),
        glm::cos( glm::radians(pitch + deltaPitch) ) * glm::sin( glm::radians(yaw + deltaYaw) )
    ) );

    cameraRightDirection = glm::normalize( glm::cross( upDirection, cameraDirection ) );

    // cameraPosition += ( cameraMoveFront * cameraDirection + cameraMoveRight * cameraRightDirection + cameraMoveUp * upDirection ) * cameraSpeed;

    gluLookAt( cameraPosition.x, cameraPosition.y, cameraPosition.z,   // Eye position
               cameraPosition.x + cameraDirection.x, cameraPosition.y + cameraDirection.y,
               cameraPosition.z + cameraDirection.z,  // Target Position
               upDirection.x, upDirection.y, upDirection.z );  // Up vector

    if( sceneType == 'o' ) {
        glBegin( GL_TRIANGLES );

        if( normals_available ) {

            float cosTheta;
            glm::vec3 vertexColor;

            for( int m = 0; m < noOfMeshes; ++m ) {
                for( int i = 0; i < obj[m].Faces.size(); ++i ) {

                    cosTheta = glm::dot( obj[m].Normals[ obj[m].Faces[i].x ], lightDirection );
                    cosTheta = glm::clamp( cosTheta, 0.0f, 1.0f );
                    vertexColor = ( lightColor * objectColorDiffuse * cosTheta )
                                + ( objectColorAmbient ) ;
                    glColor3f( vertexColor.x, vertexColor.y, vertexColor.z );
                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].x ].x,
                                obj[m].Vertices[ obj[m].Faces[i].x ].y,
                                obj[m].Vertices[ obj[m].Faces[i].x ].z );

                    cosTheta = glm::dot( obj[m].Normals[ obj[m].Faces[i].y ], lightDirection );
                    cosTheta = glm::clamp( cosTheta, 0.0f, 1.0f );
                    vertexColor = ( lightColor * objectColorDiffuse * cosTheta )
                                + ( objectColorAmbient ) ;
                    glColor3f( vertexColor.x, vertexColor.y, vertexColor.z );
                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].y ].x,
                                obj[m].Vertices[ obj[m].Faces[i].y ].y,
                                obj[m].Vertices[ obj[m].Faces[i].y ].z );

                    cosTheta = glm::dot( obj[m].Normals[ obj[m].Faces[i].z ], lightDirection );
                    cosTheta = glm::clamp( cosTheta, 0.0f, 1.0f );
                    vertexColor = ( lightColor * objectColorDiffuse * cosTheta )
                                + ( objectColorAmbient ) ;
                    glColor3f( vertexColor.x, vertexColor.y, vertexColor.z );
                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].z ].x,
                                obj[m].Vertices[ obj[m].Faces[i].z ].y,
                                obj[m].Vertices[ obj[m].Faces[i].z ].z );
                }
            }
        } // if( normals_available )
        else {

            float cosTheta;
            glm::vec3 faceColor;

            for( int m = 0; m < noOfMeshes; ++m ) {
                for( int i = 0; i < obj[m].Faces.size(); ++i ) {

                    glm::vec3 face_normal = glm::normalize( glm::cross(
                        obj[m].Vertices[obj[m].Faces[i].y] - obj[m].Vertices[obj[m].Faces[i].x],
                        obj[m].Vertices[obj[m].Faces[i].z] - obj[m].Vertices[obj[m].Faces[i].x]
                    ) );
                    cosTheta = glm::abs( glm::dot( face_normal, lightDirection ) );
                    cosTheta = glm::clamp( glm::abs(cosTheta), 0.0f, 1.0f );
                    faceColor = ( lightColor * objectColorDiffuse * cosTheta )
                                + ( objectColorAmbient ) ;
                    glColor3f( faceColor.x, faceColor.y, faceColor.z );

                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].x ].x,
                                obj[m].Vertices[ obj[m].Faces[i].x ].y,
                                obj[m].Vertices[ obj[m].Faces[i].x ].z );

                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].y ].x,
                                obj[m].Vertices[ obj[m].Faces[i].y ].y,
                                obj[m].Vertices[ obj[m].Faces[i].y ].z );

                    glVertex3f( obj[m].Vertices[ obj[m].Faces[i].z ].x,
                                obj[m].Vertices[ obj[m].Faces[i].z ].y,
                                obj[m].Vertices[ obj[m].Faces[i].z ].z );
                }
            }
        } // if( normals_available ) {...} else

        glEnd();

    } else if( sceneType == 'p' ) {
        glm::vec3 vertexColor = ( lightColor * objectColorDiffuse ) + ( objectColorAmbient ) ;
        glColor3f( vertexColor.x, vertexColor.y, vertexColor.z );

        for( int i = 0; i < pointCloud.size(); ++i ) {
            glTranslatef( pointCloud[i].x, pointCloud[i].y, pointCloud[i].z );
            glutSolidSphere( 0.03, 7, 5 );
            glTranslatef( -pointCloud[i].x, -pointCloud[i].y, -pointCloud[i].z );
        }
    }
}


void reshapeCallback( int W, int H )
{
    if( H == 0 )
        H = 1;
    float aspect_ratio = W / ( float )H;

    WIDTH = W;
    HEIGHT = H;

    glMatrixMode( GL_PROJECTION );
    glViewport( 0, 0, W, H );
    glLoadIdentity();
    gluPerspective( 45.0f, aspect_ratio, 1.0f, 1000.0f );
}


void keyboardCallback( unsigned char key, int x, int y )
{
    switch( key ) {
        case 27 /* ESC */:
        case 'q' :
            exit( EXIT_SUCCESS );
            break;
        case 'd' /*Depth Image*/ :
        case 'c' /*Color Image*/ :
            // screenshot( key );
            break;
        default:
            return;
    }
    glutPostRedisplay();
}


void keyboardSpecialDownCallback( int key, int x, int y )
{
    switch( key ) {
        case GLUT_KEY_UP :
            cameraPosition += cameraDirection;
            break;
        case GLUT_KEY_DOWN :
            cameraPosition -= cameraDirection;
            break;
        case GLUT_KEY_LEFT :
            cameraPosition += cameraRightDirection;
            break;
        case GLUT_KEY_RIGHT :
            cameraPosition -= cameraRightDirection;
            break;
        case GLUT_KEY_PAGE_UP :
            cameraPosition += upDirection;
            break;
        case GLUT_KEY_PAGE_DOWN :
            cameraPosition -= upDirection;
            break;
        default :
            return;
    }
    glutPostRedisplay();
}


void mouseButtonCallback( int button, int state, int x, int y )
{
    if( button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN ) {

        unsigned char colorPixels[ 1 * 1 * 4 ];
        glReadPixels( x, glutGet( GLUT_WINDOW_HEIGHT ) - y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
                      &colorPixels[0] );

        float depthPixels[ 1 * 1 ];
        glReadPixels( x, glutGet( GLUT_WINDOW_HEIGHT ) - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT,
                      &depthPixels[0] );

        GLint viewport[4];
        GLdouble modelView[16];
        GLdouble projection[16];
        GLfloat winX, winY, winZ;
        GLdouble worldX, worldY, worldZ;

        glGetDoublev( GL_MODELVIEW_MATRIX, modelView );
        glGetDoublev( GL_PROJECTION_MATRIX, projection );
        glGetIntegerv( GL_VIEWPORT, viewport );

        winX = float( x );
        winY = float( glutGet( GLUT_WINDOW_HEIGHT ) - y );
        winZ = depthPixels[0];

        gluUnProject( winX, winY, winZ,
                      modelView, projection, viewport,
                      &worldX, &worldY, &worldZ );

        cout << endl;
        cout << "RGBA : (" << ( int )colorPixels[0] << ", " << ( int )colorPixels[1] << ", " <<
             ( int )colorPixels[2] << ", " << ( int )colorPixels[3] << ")" << endl;
        // cout << "Depth : " << depthPixels[0] << endl;
        // cout << "Image coordinates : ( " << winX << ", " << winY << ", " << winZ << " )" << endl;
        cout << "World Coordinates : ( " << worldX << ", " << worldY << ", " << worldZ << " )"
             << endl;
    } else if( button == GLUT_LEFT_BUTTON ) {
        if( state == GLUT_DOWN ) {
            lastMousePosX = x;
            lastMousePosY = y;
        } else { // state == GLUT_UP
            lastMousePosX = -1;
            lastMousePosY = -1;
            yaw += deltaYaw;
            pitch += deltaPitch;
            deltaYaw = 0;
            deltaPitch = 0;
        }
    }

    glutPostRedisplay();
}


void mouseMotionCallback( int x, int y )
{
    if( lastMousePosX >= 0 ) {
        deltaYaw = ( lastMousePosX - x ) * mouseSensetivity;
        deltaPitch = ( y - lastMousePosY ) * mouseSensetivity;
    }
    glutPostRedisplay();
}


void loadScene_OBJ( Fl_Widget* w, void* data )
{
    Fl_File_Chooser file( ".", "OBJ FIle (*.obj)", Fl_File_Chooser::SINGLE, "File Browser" );
    file.show();

    while( file.shown() )
        Fl::wait();
    if( file.value() == NULL ) {
        fl_message_title( "Error" );
        fl_alert( "No file selected" );
        return;
    }

    string fileName = file.value();

    ifstream fin( fileName );
    if( !fin.good() ) {
        fl_message_title( "Error" );
        fl_alert( ( "Unable to read file " + fileName ).c_str() );
        return;
    } // if( !fin.good() )
    fin.close();

    cout << endl << "INFO: loadScene_OBJ: Loading file " << fileName;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile( fileName,
                           aiProcess_Triangulate           |
                           aiProcess_GenSmoothNormals      |
                           aiProcess_FixInfacingNormals    |
                           aiProcess_JoinIdenticalVertices );

    if( !scene ) {
        cout << "ERROR::ASSIMP::" << importer.GetErrorString() << endl;
        exit( EXIT_FAILURE );
    }

    noOfMeshes = scene -> mNumMeshes;
    obj = new object[ noOfMeshes ];

    aiMesh** mesh = scene -> mMeshes;

    cout << endl;
    cout << "Total no. of Meshes : " << noOfMeshes << endl;

    for( int m = 0; m < noOfMeshes; ++m ) {

        cout << endl;
        cout << "Mesh " << m + 1 << " : " << endl;
        cout << "  Faces: " << mesh[m] -> mNumFaces << endl;
        cout << "  Vertices: " << mesh[m] -> mNumVertices << endl;
        cout << "  Normals available: " << mesh[m] -> HasNormals() << endl;

        aiFace* faces = mesh[m] -> mFaces;
        for( int i = 0; i < ( mesh[m]->mNumFaces ); ++i ) {
            obj[m].Faces.push_back( glm::vec3( faces[i].mIndices[0],
                                               faces[i].mIndices[1],
                                               faces[i].mIndices[2] ) );
        }

        aiVector3D* vertices = mesh[m] -> mVertices;
        obj[m].pCorner.x = vertices[0].x;
        obj[m].pCorner.y = vertices[0].y;
        obj[m].pCorner.z = vertices[0].z;
        obj[m].nCorner.x = vertices[0].x;
        obj[m].nCorner.y = vertices[0].y;
        obj[m].nCorner.z = vertices[0].z;
        for( int i = 0; i < ( mesh[m]->mNumVertices ); ++i ) {
            obj[m].Vertices.push_back( glm::vec3( vertices[i].x, vertices[i].y, vertices[i].z ) );

            if( obj[m].pCorner.x < vertices[i].x )
                obj[m].pCorner.x = vertices[i].x;
            if( obj[m].pCorner.y < vertices[i].y )
                obj[m].pCorner.y = vertices[i].y;
            if( obj[m].pCorner.z < vertices[i].z )
                obj[m].pCorner.z = vertices[i].z;
            if( obj[m].nCorner.x > vertices[i].x )
                obj[m].nCorner.x = vertices[i].x;
            if( obj[m].nCorner.y > vertices[i].y )
                obj[m].nCorner.y = vertices[i].y;
            if( obj[m].nCorner.z > vertices[i].z )
                obj[m].nCorner.z = vertices[i].z;
        }

        aiVector3D* normals = mesh[m] -> mNormals;
        for( int i = 0; i < ( mesh[m]->mNumVertices ); ++i ) {
            obj[m].Normals.push_back( glm::normalize( glm::vec3( normals[i].x,
                                      normals[i].y,
                                      normals[i].z ) ) );
        }

    }

    BB_pCorner = obj[0].pCorner;
    BB_nCorner = obj[0].nCorner;

    for( int i = 1; i < noOfMeshes; ++i ) {
        if( BB_pCorner.x < obj[i].pCorner.x )
            BB_pCorner.x = obj[i].pCorner.x;
        if( BB_pCorner.y < obj[i].pCorner.y )
            BB_pCorner.y = obj[i].pCorner.y;
        if( BB_pCorner.z < obj[i].pCorner.z )
            BB_pCorner.z = obj[i].pCorner.z;
        if( BB_nCorner.x > obj[i].nCorner.x )
            BB_nCorner.x = obj[i].nCorner.x;
        if( BB_nCorner.y > obj[i].nCorner.y )
            BB_nCorner.y = obj[i].nCorner.y;
        if( BB_nCorner.z > obj[i].nCorner.z )
            BB_nCorner.z = obj[i].nCorner.z;
    }

    cout << endl;
    cout << "Bounding Box :" << endl;
    cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z << endl;
    cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z << endl;

    cameraPosition = glm::vec3( BB_pCorner.x + ( BB_pCorner.x - BB_nCorner.x ) / 2,
                                BB_pCorner.y + ( BB_pCorner.y - BB_nCorner.y ) / 2,
                                BB_pCorner.z + ( BB_pCorner.z - BB_nCorner.z ) / 2 );
    float   distX = BB_pCorner.x - BB_nCorner.x,
            distY = BB_pCorner.y - BB_nCorner.y,
            distZ = BB_pCorner.z - BB_nCorner.z,
            distXZ = glm::sqrt( pow( distX, 2 ) + pow( distZ, 2 ) ),
            distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
    pitch =  glm::degrees( -( glm::atan( distY, distXZ ) ) );
    yaw = 180 + glm::degrees( glm::atan( distZ, distX ) );

    cameraSpeed = distXYZ / 200.0;

    cout << endl;
    cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y << " " <<
         cameraPosition.z << endl;

    sceneType = 'o';

    glutPostRedisplay();

}


void loadScene_PLY( Fl_Widget* w, void* data )
{
    Fl_File_Chooser file( ".", "PLY FIle (*.ply)", Fl_File_Chooser::SINGLE, "File Browser" );
    file.show();

    while( file.shown() )
        Fl::wait();
    if( file.value() == NULL ) {
        // cout << "No file selected" << endl;
        fl_message_title( "Error" );
        fl_alert( "No file selected" );
        return;
    }

    string fileName = file.value();

    ifstream fin( fileName );
    if( !fin.good() ) {
        fl_message_title( "Error" );
        fl_alert( ( "Unable to read file " + fileName).c_str() );
        return;
    } // if( !fin.good() )

    cout << endl << "INFO: loadScene_PLY: Loading file " << fileName;

    noOfMeshes = 1;
    obj = new object[noOfMeshes];

    int numFaces, numVertices;
    string inputString;
    fin >> inputString;
    while( inputString.compare( "end_header" ) != 0 ) {

        fin >> inputString;
        if( inputString.compare( "element" ) == 0 ) {
            fin >> inputString;
            if( inputString.compare( "vertex" ) == 0 ) {
                fin >> numVertices;
            } else if( inputString.compare( "face" ) == 0 ) {
                fin >> numFaces;
            }
        } else if( inputString.compare( "property" ) == 0 ) {
            fin >> inputString;
            fin >> inputString;
            if( inputString.compare( "nx" ) == 0 || inputString.compare( "ny" ) == 0
                    || inputString.compare( "nz" ) == 0 ) {
                normals_available = true;
            }
        }

    }

    // if( !normals_available ) {
    //     fl_message_title( "Error" );
    //     fl_alert( ( "file " + fileName + " does not contain Normals!").c_str() );
    //     return;
    // }

    cout << endl;
    cout << "Faces: " << numFaces << endl;
    cout << "Vertices: " << numVertices << endl;

    obj->Vertices.clear();
    obj->Faces.clear();
    obj->Normals.clear();

    double x, y, z;
    for( long int i = 0; i < numVertices; ++i ) {
        fin >> x >> y >> z;
        obj->Vertices.emplace_back( x, y, z );

        if( normals_available ) {
            fin >> x >> y >> z;
            obj->Normals.emplace_back( x, y, z );
        }
    }
    long int faceSize;
    for( long int i = 0; i < numFaces; ++i ) {
        fin >> faceSize;
        fin >> x >> y >> z ;
        if( faceSize != 3 ) {
            cerr << "ERROR: loadScene_PLY: faceSize is " << faceSize << " instead of 3!"
                      << endl;
            exit( EXIT_SUCCESS );
        }
        obj->Faces.emplace_back( x, y, z );
    }

    fin.close();

    obj->pCorner.x = obj->Vertices[0].x;
    obj->pCorner.y = obj->Vertices[0].y;
    obj->pCorner.z = obj->Vertices[0].z;
    obj->nCorner.x = obj->Vertices[0].x;
    obj->nCorner.y = obj->Vertices[0].y;
    obj->nCorner.z = obj->Vertices[0].z;
    for( int i = 1; i < numVertices; ++i ) {
        obj->pCorner.x = max( obj->pCorner.x, obj->Vertices[i].x );
        obj->pCorner.y = max( obj->pCorner.y, obj->Vertices[i].y );
        obj->pCorner.z = max( obj->pCorner.z, obj->Vertices[i].z );
        obj->nCorner.x = min( obj->nCorner.x, obj->Vertices[i].x );
        obj->nCorner.y = min( obj->nCorner.y, obj->Vertices[i].y );
        obj->nCorner.z = min( obj->nCorner.z, obj->Vertices[i].z );
    }
    BB_pCorner.x = max( BB_pCorner.x, obj->pCorner.x );
    BB_pCorner.y = max( BB_pCorner.y, obj->pCorner.y );
    BB_pCorner.z = max( BB_pCorner.z, obj->pCorner.z );
    BB_nCorner.x = min( BB_nCorner.x, obj->nCorner.x );
    BB_nCorner.y = min( BB_nCorner.y, obj->nCorner.y );
    BB_nCorner.z = min( BB_nCorner.z, obj->nCorner.z );

    cout << endl;
    cout << "Bounding Box :" << endl;
    cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z << endl;
    cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z << endl;

    cameraPosition = glm::vec3( BB_pCorner.x + ( BB_pCorner.x - BB_nCorner.x ) / 2,
                                BB_pCorner.y + ( BB_pCorner.y - BB_nCorner.y ) / 2,
                                BB_pCorner.z + ( BB_pCorner.z - BB_nCorner.z ) / 2 );
    float   distX = BB_pCorner.x - BB_nCorner.x,
            distY = BB_pCorner.y - BB_nCorner.y,
            distZ = BB_pCorner.z - BB_nCorner.z,
            distXZ = glm::sqrt( pow( distX, 2 ) + pow( distZ, 2 ) ),
            distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
    pitch =  glm::degrees( -( glm::atan( distY, distXZ ) ) );
    yaw = 180 + glm::degrees( glm::atan( distZ, distX ) );

    cameraSpeed = distXYZ / 200.0;

    cout << endl;
    cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y << " " <<
         cameraPosition.z << endl;

    sceneType = 'o';

    glutPostRedisplay();

}


void loadPointCloud_OBJ( Fl_Widget* w, void* data )
{
    Fl_File_Chooser file( ".", "OBJ FIle (*.obj)", Fl_File_Chooser::SINGLE, "File Browser" );
    file.show();

    while( file.shown() )
        Fl::wait();
    if( file.value() == NULL ) {
        // cout << "No file selected" << endl;
        fl_message_title( "Error" );
        fl_alert( "No file selected" );
        return;
    }

    string fileName = file.value();

    ifstream fin( fileName );
    if( !fin.good() ) {
        fl_message_title( "Error" );
        fl_alert( ( "Unable to read file " + fileName).c_str() );
        return;
    } // if( !fin.good() )

    cout << endl << "INFO: loadPointCloud_OBJ: Loading file " << fileName;

    char c;
    GLfloat x, y, z;

    BB_pCorner.x = -1e3;
    BB_pCorner.y = -1e3;
    BB_pCorner.z = -1e3;
    BB_nCorner.x = 1e3;
    BB_nCorner.y = 1e3;
    BB_nCorner.z = 1e3;

    while( fin >> c >> x >> y >> z ) {
        pointCloud.push_back( glm::vec3( x, y, z ) );
        if( BB_pCorner.x < x ) BB_pCorner.x = x;
        if( BB_pCorner.y < y ) BB_pCorner.y = y;
        if( BB_pCorner.z < z ) BB_pCorner.z = z;
        if( BB_nCorner.x > x ) BB_nCorner.x = x;
        if( BB_nCorner.y > y ) BB_nCorner.y = y;
        if( BB_nCorner.z > z ) BB_nCorner.z = z;
    }

    pointCloud.shrink_to_fit();

    fin.close();

    cout << endl;
    cout << "Bounding Box :" << endl;
    cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z << endl;
    cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z << endl;

    cameraPosition = glm::vec3( BB_pCorner.x + ( BB_pCorner.x - BB_nCorner.x ) / 2,
                                BB_pCorner.y + ( BB_pCorner.y - BB_nCorner.y ) / 2,
                                BB_pCorner.z + ( BB_pCorner.z - BB_nCorner.z ) / 2 );
    float   distX = BB_pCorner.x - BB_nCorner.x,
            distY = BB_pCorner.y - BB_nCorner.y,
            distZ = BB_pCorner.z - BB_nCorner.z,
            distXZ = glm::sqrt( pow( distX, 2 ) + pow( distZ, 2 ) ),
            distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
    pitch =  glm::degrees( -( glm::atan( distY, distXZ ) ) );
    yaw = 180 + glm::degrees( glm::atan( distZ, distX ) );

    cameraSpeed = distXYZ / 200.0;

    cout << endl;
    cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y << " "
         << cameraPosition.z << endl;

    sceneType = 'p';

    glutPostRedisplay();
}


void loadPointCloud_PTCLD( Fl_Widget* w, void* data )
{
    Fl_File_Chooser file( ".", "PTCLD FIle (*.ptcld)", Fl_File_Chooser::SINGLE, "File Browser" );
    file.show();

    while( file.shown() )
        Fl::wait();
    if( file.value() == NULL ) {
        // cout << "No file selected" << endl;
        fl_message_title( "Error" );
        fl_alert( "No file selected" );
        return;
    }

    string fileName = file.value();

    ifstream fin( fileName );
    if( !fin.good() ) {
        fl_message_title( "Error" );
        fl_alert( ( "Unable to read file " + fileName).c_str() );
        return;
    } // if( !fin.good() )

    cout << endl << "INFO: loadPointCloud_PTCLD: Loading file " << fileName << endl;

     // Load file header, to be discarded
    string file_header;
    getline( fin, file_header );

    // Load vertices and their correnponding vertex index
    // The file is in the form:
    //   pt.x pt.y pt.z idx
    float x, y, z;
    int i;

    BB_pCorner.x = -1e3;
    BB_pCorner.y = -1e3;
    BB_pCorner.z = -1e3;
    BB_nCorner.x = 1e3;
    BB_nCorner.y = 1e3;
    BB_nCorner.z = 1e3;
    while( fin >> x >> y >> z >> i ) {
        pointCloud.push_back( glm::vec3( x, y, z ) );
        if( BB_pCorner.x < x ) BB_pCorner.x = x;
        if( BB_pCorner.y < y ) BB_pCorner.y = y;
        if( BB_pCorner.z < z ) BB_pCorner.z = z;
        if( BB_nCorner.x > x ) BB_nCorner.x = x;
        if( BB_nCorner.y > y ) BB_nCorner.y = y;
        if( BB_nCorner.z > z ) BB_nCorner.z = z;
    }

    pointCloud.shrink_to_fit();

    fin.close();

    cout << endl;
    cout << "Bounding Box :" << endl;
    cout << "  Corner 1 : " << BB_pCorner.x << " " << BB_pCorner.y << " " << BB_pCorner.z << endl;
    cout << "  Corner 2 : " << BB_nCorner.x << " " << BB_nCorner.y << " " << BB_nCorner.z << endl;

    cameraPosition = glm::vec3( BB_pCorner.x + ( BB_pCorner.x - BB_nCorner.x ) / 2,
                                BB_pCorner.y + ( BB_pCorner.y - BB_nCorner.y ) / 2,
                                BB_pCorner.z + ( BB_pCorner.z - BB_nCorner.z ) / 2 );
    float   distX = BB_pCorner.x - BB_nCorner.x,
            distY = BB_pCorner.y - BB_nCorner.y,
            distZ = BB_pCorner.z - BB_nCorner.z,
            distXZ = glm::sqrt( pow( distX, 2 ) + pow( distZ, 2 ) ),
            distXYZ = glm::sqrt( pow( distXZ, 2 ) + pow( distY, 2 ) );
    pitch =  glm::degrees( -( glm::atan( distY, distXZ ) ) );
    yaw = 180 + glm::degrees( glm::atan( distZ, distX ) );

    cameraSpeed = distXYZ / 200.0;

    cout << endl;
    cout << "Camera Initial Position : " << cameraPosition.x << " " << cameraPosition.y << " "
         << cameraPosition.z << endl;

    sceneType = 'p';

    glutPostRedisplay();
}


void screenshot( Fl_Widget* w, void* data )
{
    // TYPE INFORMATION
    //   d : Depth
    //   c : Color (RGB)

    char* temp = (char*) data;
    char type = temp[0];

    int W = WIDTH, // glutGet(GLUT_WINDOW_WIDTH),
        H = HEIGHT; // glutGet(GLUT_WINDOW_HEIGHT);

    ofstream fout;

    if( type == 'd' ) {
        Fl_File_Chooser file( ".", "PGM FIle (*.pgm)", Fl_File_Chooser::CREATE, "File Browser" );
        file.show();

        while( file.shown() )
            Fl::wait();
        if( file.value() == NULL ) {
            // cout << "No file selected" << endl;
            fl_message_title( "Error" );
            fl_alert( "No file selected" );
            return;
        }

        fout.open( file.value() );

        float* depthPixels = new float[ W * H ];
        glReadPixels( 0, 0, W, H, GL_DEPTH_COMPONENT, GL_FLOAT, &depthPixels[0] );

        fout << "P2" << endl << W << " " << H << endl << 255 << endl;
        for( int h = 0; h < H; ++h ) {
            for( int w = 0; w < W; ++w ) {
                fout << int( depthPixels[( H - h - 1 )*W + w] * 255 ) << " ";
            }
            fout << endl;
        }

        cout << "INFO: Depth image written : " << file.value() << endl;

    } // if( type == 'd' )
    else if( type == 'c' ) {
        Fl_File_Chooser file( ".", "PPM FIle (*.pPm)", Fl_File_Chooser::CREATE, "File Browser" );
        file.show();

        while( file.shown() )
            Fl::wait();
        if( file.value() == NULL ) {
            // cout << "No file selected" << endl;
            fl_message_title( "Error" );
            fl_alert( "No file selected" );
            return;
        }

        fout.open( file.value() );

        unsigned char* colorPixels = new unsigned char[ W * H * 3 ];
        glReadPixels( 0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, &colorPixels[0] );

        fout << "P3" << endl << W << " " << H << endl << 255 << endl;
        for( int h = 0; h < H; ++h ) {
            for( int w = 0; w < W; ++w )
                fout << int( colorPixels[ ( (H-h-1) * W + w ) * 3 ] ) << " "
                     << int( colorPixels[ ( (H-h-1) * W + w ) * 3 + 1 ] ) << " "
                     << int( colorPixels[ ( (H-h-1) * W + w ) * 3 + 2 ] ) << "   ";
            fout << endl;
        }

        cout << "INFO: Color image \"image_rgb.ppm\" saved!" << endl;

    } // if( type == 'd' ) {...} else if( type == 'd' )
    else if( type == 'p' ) {
        Fl_File_Chooser file( ".", "OBJ FIle (*.obj)", Fl_File_Chooser::CREATE, "File Browser" );
        file.show();

        while( file.shown() )
            Fl::wait();
        if( file.value() == NULL ) {
            // cout << "No file selected" << endl;
            fl_message_title( "Error" );
            fl_alert( "No file selected" );
            return;
        } else {
            cout << endl;
            cout << "Point Cloud written : " << file.value() << endl;
        }

        fout.open( file.value() );

        float* depthPixels = new float[ W * H ];
        glReadPixels( 0, 0, W, H, GL_DEPTH_COMPONENT, GL_FLOAT, &depthPixels[0] );

        GLint viewport[4];
        GLdouble modelView[16];
        GLdouble projection[16];
        GLfloat winX, winY, winZ;
        GLdouble worldX, worldY, worldZ;

        glGetDoublev( GL_MODELVIEW_MATRIX, modelView );
        glGetDoublev( GL_PROJECTION_MATRIX, projection );
        glGetIntegerv( GL_VIEWPORT, viewport );

        for( int h = 0; h < H; ++h ) {
            for( int w = 0; w < W; ++w ) {
                winX = float( w );
                winY = float( glutGet( GLUT_WINDOW_HEIGHT ) - h );
                winZ = depthPixels[( H - h - 1 ) * W + w];
                gluUnProject( winX, winY, winZ,
                              modelView, projection, viewport,
                              &worldX, &worldY, &worldZ );

                if( BB_nCorner.x - 1 < worldX && worldX < BB_pCorner.x + 1
                        && BB_nCorner.y - 1 < worldY && worldY < BB_pCorner.y + 1
                        && BB_nCorner.z - 1 < worldZ && worldZ < BB_pCorner.z + 1 )
                    fout << "v " << worldX << " " << worldY << " " << worldZ << endl;
            }
        }

    } // // if( type == 'd' ) {...} else if( type == 'd' ) {...} else if( type == 'p' )

    fout.close();

}


void subset_pt_cld( Fl_Widget* w, void* data )
{
    if( noOfMeshes != 1 ) {
        fl_message_title( "Error" );
        fl_alert( "ERROR: subset_pt_cld: Number of meshes != 1" );
        return;
    }

    std::vector< std::pair< glm::vec3, unsigned int > > POINT_CLOUD_with_CORRESPONDANCE;
    int max_points = 500;
    int numVertices = obj->Vertices.size();

    /*
     * Generating subset point cloud
     */
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
    auto random_generator_uniform = std::bind( distribution, generator );

    // Generate random vertices
    set<int> visited;
    while( visited.size() < max_points ) {
        int rand_vert_idx = random_generator_uniform() * numVertices;
        if( visited.find(rand_vert_idx) == visited.end() ) { // if `rand_vert_idx` not in `visited`
            visited.insert( rand_vert_idx );
        }
    }

    // Make Point Cloud from random vertices made above
    for( set<int>::iterator it = visited.begin(); it != visited.end(); ++it ) {
        POINT_CLOUD_with_CORRESPONDANCE.push_back( make_pair( obj->Vertices[*it], *it ) );
    }


    /*
     * Output file chooser
     */
    Fl_File_Chooser file_save( ".", "Point Cloud File (*.ptcld)", Fl_File_Chooser::CREATE, "File Browser" );
    file_save.show();

    while( file_save.shown() )
        Fl::wait();
    if( file_save.value() == NULL ) {
        fl_message_title( "Error" );
        fl_alert( "No file selected" );
        return;
    }

    ofstream fout( file_save.value() );


    /*
     * Saving Output file
     */
    fout << "# " << numVertices << " " << max_points << endl;
    for( int i = 0; i < POINT_CLOUD_with_CORRESPONDANCE.size(); ++i ) {
        fout << POINT_CLOUD_with_CORRESPONDANCE[i].first.x << " " << POINT_CLOUD_with_CORRESPONDANCE[i].first.y << " "
             << POINT_CLOUD_with_CORRESPONDANCE[i].first.z << " " << POINT_CLOUD_with_CORRESPONDANCE[i].second << endl;
    }

    fout.close();
}


void editSceneParam( Fl_Widget* w, void* data )
{
    char* temp = ( char* )data;
    char type = temp[0];

    double a, b, c;

    // Light Direction
    if( type == 'd' ) {
        string s;

        fl_message_title( "Light Direction Vector" );
        s = fl_input( "x value", "1.0" );
        a = stod( s.c_str() );
        fl_message_title( "Light Direction Vector" );
        s = fl_input( "y value", "1.0" );
        b = stod( s.c_str() );
        fl_message_title( "Light Direction Vector" );
        s = fl_input( "z value", "1.0" );
        c = stod( s.c_str() );

        lightDirection = glm::normalize( glm::vec3( float( a ), float( b ), float( c ) ) );
    }
    // Light Color
    else if( type == 'c' ) {
        fl_color_chooser( "Light Color Chooser", a, b, c );
        lightColor = glm::vec3( float( a ), float( b ), float( c ) );
    }
    // Object Color
    else if( type == 'o' ) {
        fl_color_chooser( "Object Color Chooser", a, b, c );
        objectColorDiffuse = glm::vec3( float( a ), float( b ), float( c ) );
        objectColorAmbient = objectColorDiffuse * glm::vec3( 0.1f, 0.1f, 0.1f );
    }

    glutPostRedisplay();
}
