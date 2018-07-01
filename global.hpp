#ifndef HUMAN_BODY_RECONSTRUCTION_GLOBAL_HPP
#define HUMAN_BODY_RECONSTRUCTION_GLOBAL_HPP

// The below custom defined ERROR throws exception if
// GLOG_NO_ABBREVIATED_SEVERITIES is not defined
#define GLOG_NO_ABBREVIATED_SEVERITIES
#define GLM_ENABLE_EXPERIMENTAL


/*
 * Custom defined ASSERT, ERROR and WARNING
 */
#define ASSERT(condition, message)												\
    if (! (condition)) {														\
        std::cerr << "Assertion `" #condition "` failed in " << __FILE__		\
                  << " line " << __LINE__ << ": " << message << std::endl;		\
        std::exit( EXIT_SUCCESS );												\
    }

#define ERROR(message)															\
{																				\
    std::cerr << "ERROR [" << __FUNCTION__ << "] : " << message << std::endl;	\
    std::exit( EXIT_SUCCESS ); 													\
}


#define WARNING(message)														\
    std::cerr << "WARNING [" << __FUNCTION__ << "] : " << message << std::endl;


/*
 ***********************************************************************
 ***********************************************************************
 *
 * Header Files
 *
 ***********************************************************************
 ***********************************************************************
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <utility>
#include <string>
#include <random>
#include <utility> // std::pair
#include <functional> // std::bind
#include <cmath>

#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <armadillo>
#include <pca.h>
#include <ceres/ceres.h>


/*
 ***********************************************************************
 ***********************************************************************
 *
 * Function & Structure Signatures
 *
 ***********************************************************************
 ***********************************************************************
 */

struct Mesh;


void displayCallback();
void reshapeCallback( int new_width, int new_height );
void keyboardCallback( unsigned char key, int x, int y );
void keyboardSpecialDownCallback( int key, int x, int y );
void keyboardSpecialUpCallback( int key, int x, int y );
void mouseButtonCallback( int button, int state, int x, int y );
void mouseMotionCallback( int x, int y );

void screenshot( char type );
void loadMeshes_ASSIMP( Mesh * obj, std::string fileName );
void loadMeshes();
void loadSkeleton( std::string skeletonPartsFileName );
void saveParams();
void initData();
void processData( int meshPose, int meshShape );



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Global Variables
 *
 ***********************************************************************
 ***********************************************************************
 */

glm::vec3 cameraPosition;
glm::vec3 cameraDirection, cameraRightDirection;

/*
 * SCAPE Variables and Parameters
 */

// Bounding Box corners
glm::vec3 BB_pCorner, BB_nCorner;
long int numMeshes_POSE, numMeshes_SHAPE, numBones, numVertices, numFaces;

// Mesh is object for storing meshes vartices, faces and other things
// like Rotation matrices R, Deformation matrices Q & S.
struct Mesh {
    std::vector< glm::vec3 > Faces;
    std::vector< glm::vec3 > Vertices;
    std::vector< glm::vec3 > Normals;
    glm::vec3 pCorner, nCorner;
    // std::vector< glm::vec3 > boneDirection; // size of numBones

    // SCAPE Variables
    std::vector< glm::mat3 > matR; // sizeof numBones
    std::vector< glm::mat3 > matQ; // sizeof numFaces
    std::vector< glm::mat3 > matS; // sizeof numFaces
    std::vector< double * > deltaR; // sizeof numBones X 7
} *obj_POSE, *obj_SHAPE, *template_mesh, *Y, *Y2, *Y3;

// vertBone[i] stores index of bone belonging to i'th vertex
std::vector< unsigned int > vertBone; // sizeof numVertices

// faceBone[i] stores index of bone belonging to i'th face
std::vector< unsigned int > faceBone; // sizeof numFaces

// boneFaces[i] is a list of all the faces belonging to i'th bone
std::vector< std::vector< unsigned int > > boneFaces; // sizeof numBones

// boneColor[i] is a 3D vector storing the color of i'th bone
// boneColor[i] looks like this: [0.49, 0.77, 0.12]
std::vector< glm::vec3 > boneColor; // sizeof numBones

// joinedBones[i] is a 2D vector storing the indices of bones connected
// to i'th bone
// For ex, if joinedBones[2] = [4, 7]
// 2nd bone is connected to 4th abd 7t bone
std::vector< glm::ivec2 > joinedBones; // sizeof numBones

// Regression vector A used to estimate matQ
std::vector< double ** > A; // sizeof numFaces X 9 X 7

// PCA parapeters matU, vecMu and vecSigma are used to estimate matS
arma::mat matU; // sizeof numMeshes_SHAPE X ( numFaces * 9 )
arma::vec vecMu; // sizeof ( numFaces * 9 )
arma::vec vecSigma; // sizeof numMeshes_SHAPE

// lightColor and lightFirection are defined for rendering the meshes
// Not used currently
glm::vec3 lightColor = glm::vec3( 1.0f, 1.0f, 1.0f );
glm::vec3 lightDirection = glm::normalize( glm::vec3( 1.0f, 1.0f, 1.0f ) );

// DATA_DIR is the directory of the data
// This directory stores meshes, vecA and PCA parameters
std::string DATA_DIR;

// POINT_CLOUD is a list of 3D points, to do the mesh estimation upon
// POINT_CLOUD[i].first = 3D coordinated of the i'th point
// POINT_CLOUD[i].second = (correspondance) index of corresponding vertex
std::vector< std::pair< glm::vec3, unsigned int > > POINT_CLOUD;


// Initial meshes were not alighed to the same direction
// This was used to align meshes to the same direction
// Not required anymore
bool ALIGN_MESHES_ALONG_Y_AXIS     = false;

// Bool variables to load or compute Rotation matrices matR
bool LOAD_R_FROM_DISK              = true;
bool SAVE_R_TO_DISK                = false;

// Bool variables to load or compute
// Pose-Space Deformation matrices matQ
bool LOAD_Q_FROM_DISK              = true;
bool SAVE_Q_TO_DISK                = false;

// Bool variables to load or compute
// Shape-Space Deformation matrices matS
bool LOAD_S_FROM_DISK              = true;
bool SAVE_S_TO_DISK                = false; // Not used

// Bool variables to load or compute
// regression vector A
bool LOAD_A_FROM_DISK              = true;
bool SAVE_A_TO_DISK                = false;

// Bool variables to load or compute
// PCA parameters matU, vecMu, vecSigma
bool LOAD_U_Mu_Sigma_FROM_DISK     = true;
bool SAVE_U_Mu_Sigma_TO_DISK       = false; // Not used



#endif //HUMAN_BODY_RECONSTRUCTION_GLOBAL_HPP
