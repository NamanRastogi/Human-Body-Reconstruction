#ifndef HUMAN_BODY_RECONSTRUCTION_FILE_OPERATIONS_HPP
#define HUMAN_BODY_RECONSTRUCTION_FILE_OPERATIONS_HPP


/*
 ***********************************************************************
 ***********************************************************************
 *
 * Header Files
 *
 ***********************************************************************
 ***********************************************************************
 */

#include "global.hpp"



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Function Definitions
 *
 ***********************************************************************
 ***********************************************************************
 */


/*
 * Function to take screenshot of the current OpenGL rendered window
 */
void screenshot( char type )
{

    // TYPE INFORMATION
    //   d : Depth
    //   c : Color (RGB)

    int W = glutGet( GLUT_WINDOW_WIDTH ),
        H = glutGet( GLUT_WINDOW_HEIGHT );

    // Save Depth Images as a Point Cloud as well as Depth Image
    // But this Point Cloud cannot be used in Shape Completion
    if( type == 'd' ) {

        std::ofstream fout1, fout2;
        fout1.open( "image_depth.pgm" );
        fout2.open( "point_cloud.obj" );

        float *depthPixels = new float[ W * H ];

        // Load depth image into depthPixels
        glReadPixels( 0, 0, W, H, GL_DEPTH_COMPONENT, GL_FLOAT, &depthPixels[0] );

        GLint viewport[4];
        GLdouble modelView[17];
        GLdouble projection[17];
        GLfloat winX, winY, winZ;
        GLdouble worldX, worldY, worldZ;

        // Load Model-View matrix and Projection Matrix
        glGetDoublev( GL_MODELVIEW_MATRIX, modelView );
        glGetDoublev( GL_PROJECTION_MATRIX, projection );
        glGetIntegerv( GL_VIEWPORT, viewport );

        fout1 << "P2" << std::endl << W << " " << H << std::endl << 255 << std::endl;
        for( int h = 0; h < H; ++h ) {
            for( int w = 0; w < W; ++w ) {
                fout1 << int( depthPixels[( H - h - 1 )*W + w] * 255 ) << " ";

                winX = float( w );
                winY = float( glutGet( GLUT_WINDOW_HEIGHT ) - h );
                winZ = depthPixels[( H - h - 1 ) * W + w];
                gluUnProject( winX, winY, winZ,
                              modelView, projection, viewport,
                              &worldX, &worldY, &worldZ );

                if( BB_nCorner.x - 1 < worldX && worldX < BB_pCorner.x + 1
                        && BB_nCorner.y - 1 < worldY && worldY < BB_pCorner.y + 1
                        && BB_nCorner.z - 1 < worldZ && worldZ < BB_pCorner.z + 1 )
                    fout2 << "v " << worldX << " " << worldY << " " << worldZ << std::endl;
            }
            fout1 << std::endl;
        }

        fout1.close();
        fout2.close();

        std::cout << std::endl;
        std::cout << "Depth image \"image_depth.pgm\" saved!" << std::endl;
        std::cout << "Point Cloud saved in \"point_cloud.obj\"" << std::endl;

    }
    // Save Color Image
    else if( type == 'c' ) {

        std::ofstream fout;
        fout.open( "image_rgb.ppm" );

        unsigned char *colorPixels = new unsigned char[ W * H * 3 ];

        // Load  Color image into colorPixels
        glReadPixels( 0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, &colorPixels[0] );

        fout << "P3" << std::endl << W << " " << H << std::endl << 255 << std::endl;
        for( int h = 0; h < H; ++h ) {
            for( int w = 0; w < W; ++w )
                fout << int( colorPixels[( ( H - h - 1 )*W + w ) * 3 ] ) << " "
                     << int( colorPixels[( ( H - h - 1 )*W + w ) * 3 + 1 ] ) << " "
                     << int( colorPixels[( ( H - h - 1 )*W + w ) * 3 + 2 ] ) << "   ";
            fout << std::endl;
        }

        fout.close();

        std::cout << "Color image \"image_rgb.ppm\" saved!" << std::endl;

    }

}


/*
 * Load meshes using ASSIMP library, but is no longer required
 *
 * This function is required if there are multiple objects in a single
 * OBJ file
 */
void loadMeshes_ASSIMP( Mesh * obj, std::string fileName )
{
    // Initialize ASSIMP
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile( fileName,
                           aiProcess_Triangulate           |
                           aiProcess_GenSmoothNormals      |
                           aiProcess_FixInfacingNormals    |
                           aiProcess_JoinIdenticalVertices );

    // Error handling in importing the data from file
    if( !scene ) {
        ERROR( importer.GetErrorString() );
        exit( EXIT_FAILURE );
    }

    numMeshes_POSE = scene -> mNumMeshes;

    obj = new Mesh[ numMeshes_POSE ];

    aiMesh** mesh = scene -> mMeshes;

    std::cout << std::endl;
    std::cout << "Total no. of Meshes : " << numMeshes_POSE << std::endl;

    for( int m = 0; m < numMeshes_POSE; ++m ) {

        std::cout << std::endl;
        std::cout << "Mesh " << m + 1 << " : " << std::endl;
        std::cout << "  Faces: " << mesh[m] -> mNumFaces << std::endl;
        std::cout << "  Vertices: " << mesh[m] -> mNumVertices << std::endl;
        std::cout << "  Normals available: " << mesh[m] -> HasNormals() << std::endl;

        // Load faces from ASSIMP object
        aiFace *faces = mesh[m] -> mFaces;
        for( int i = 0; i < ( mesh[m]->mNumFaces ); ++i ) {
            obj[m].Faces.push_back( glm::vec3( faces[i].mIndices[0],
                                               faces[i].mIndices[1],
                                               faces[i].mIndices[2] ) );
        }

        // Load vertices from ASSIMP object and set BB_pCorner and
        // BB_nCorner
        aiVector3D *vertices = mesh[m] -> mVertices;
        obj[m].pCorner.x = vertices[0].x;
        obj[m].pCorner.y = vertices[0].y;
        obj[m].pCorner.z = vertices[0].z;
        obj[m].nCorner.x = vertices[0].x;
        obj[m].nCorner.y = vertices[0].y;
        obj[m].nCorner.z = vertices[0].z;
        for( int i = 0; i < ( mesh[m]->mNumVertices ); ++i ) {
            obj[m].Vertices.push_back( glm::vec3( vertices[i].x,
                                                  vertices[i].y,
                                                  vertices[i].z ) );

            if( obj[m].pCorner.x < vertices[i].x )	obj_POSE[m].pCorner.x = vertices[i].x;
            if( obj[m].pCorner.y < vertices[i].y )	obj_POSE[m].pCorner.y = vertices[i].y;
            if( obj[m].pCorner.z < vertices[i].z )	obj_POSE[m].pCorner.z = vertices[i].z;
            if( obj[m].nCorner.x > vertices[i].x )	obj_POSE[m].nCorner.x = vertices[i].x;
            if( obj[m].nCorner.y > vertices[i].y )	obj_POSE[m].nCorner.y = vertices[i].y;
            if( obj[m].nCorner.z > vertices[i].z )	obj_POSE[m].nCorner.z = vertices[i].z;
        }

        // Load nrormals from ASSIMP object
        aiVector3D *normals = mesh[m] -> mNormals;
        for( int i = 0; i < ( mesh[m]->mNumVertices ); ++i ) {
            obj[m].Normals.push_back( glm::normalize( glm::vec3( normals[i].x,
                                      normals[i].y,
                                      normals[i].z ) ) );
        }

    }

    // set overall BB_pCorner and BB_nCorner
    BB_pCorner = obj[0].pCorner;
    BB_nCorner = obj[0].nCorner;
    for( int i = 1; i < numMeshes_POSE; ++i ) {
        if( BB_pCorner.x < obj[i].pCorner.x )	BB_pCorner.x = obj_POSE[i].pCorner.x;
        if( BB_pCorner.y < obj[i].pCorner.y )	BB_pCorner.y = obj_POSE[i].pCorner.y;
        if( BB_pCorner.z < obj[i].pCorner.z )	BB_pCorner.z = obj_POSE[i].pCorner.z;
        if( BB_nCorner.x > obj[i].nCorner.x )	BB_nCorner.x = obj_POSE[i].nCorner.x;
        if( BB_nCorner.y > obj[i].nCorner.y )	BB_nCorner.y = obj_POSE[i].nCorner.y;
        if( BB_nCorner.z > obj[i].nCorner.z )	BB_nCorner.z = obj_POSE[i].nCorner.z;
    }

}


/*
 * Write mesh as PLY file
 */
void savePLY( Mesh * Y, std::string filename )
{
    std::ofstream fout( filename, std::ios::trunc );

    // Write header
    fout << "ply" << std::endl
         << "format ascii 1.0" << std::endl
         << "element vertex " << numVertices << std::endl
         << "property float x" << std::endl
         << "property float y" << std::endl
         << "property float z" << std::endl
         << "element face " << numFaces << std::endl
         << "property list uchar int vertex_indices" << std::endl
         << "end_header" << std::endl;

    // Write vertices
    for( int i = 0; i < numVertices; ++i ) {
        fout << Y->Vertices[i].x << " " << Y->Vertices[i].y
             << " " << Y->Vertices[i].z << std::endl;
    }

    // Write faces
    for( int i = 0 ; i < numFaces; ++i ) {
        fout << "3 " << template_mesh->Faces[i].x << " " << template_mesh->Faces[i].y
             << " " << template_mesh->Faces[i].z << std::endl;
    }

}


/*
 * Load mesh from PLY file
 */
void loadPLY( Mesh * M, std::string fileName, glm::mat4 Transform = glm::mat4( 1.0f ) )
{
    std::ifstream fin( fileName );
    if( !fin.good() ) {
        ERROR( "Unable to read file " + fileName );
    } // if( !fin.good() )

    // Read header
    std::string inputString;
    fin >> inputString;
    while( inputString.compare( "end_header" ) != 0 ) {

        fin >> inputString;
        if( inputString.compare( "element" ) == 0 ) {
            fin >> inputString;
            if( inputString.compare( "vertex" ) == 0 ) {
                fin >> numVertices;
                // std::cout << "No of Vertices : " << numVertices << std::endl;
            } else if( inputString.compare( "face" ) == 0 ) {
                fin >> numFaces;
                // std::cout << "No of Faces : " << numFaces << std::endl;
            }

        }

    }

    // Read vertices
    double x, y, z;
    M->Vertices.clear();
    for( long int i = 0; i < numVertices; ++i ) {
        fin >> x >> y >> z;
        // M->Vertices.emplace_back(x, y, z);
        glm::vec4 v4( x, y, z, 1.0 );
        M->Vertices.push_back( glm::vec3( Transform * v4 ) );
    }

    // Read faces
    long int faceSize;
    M->Faces.clear();
    for( long int i = 0; i < numFaces; ++i ) {
        fin >> faceSize;
        fin >> x >> y >> z ;
        ASSERT( faceSize == 3, " faceSize:" << faceSize << " (x,y,z):"
                               << "(" << x << "," << y << "," << z << ")" );
        M->Faces.emplace_back( x, y, z );
    }

    fin.close();

    // Update BB_pCorner and BB_nCorner
    M->pCorner.x = M->Vertices[0].x;
    M->pCorner.y = M->Vertices[0].y;
    M->pCorner.z = M->Vertices[0].z;
    M->nCorner.x = M->Vertices[0].x;
    M->nCorner.y = M->Vertices[0].y;
    M->nCorner.z = M->Vertices[0].z;
    for( int i = 1; i < numVertices; ++i ) {
        M->pCorner.x = std::max( M->pCorner.x, M->Vertices[i].x );
        M->pCorner.y = std::max( M->pCorner.y, M->Vertices[i].y );
        M->pCorner.z = std::max( M->pCorner.z, M->Vertices[i].z );
        M->nCorner.x = std::min( M->nCorner.x, M->Vertices[i].x );
        M->nCorner.y = std::min( M->nCorner.y, M->Vertices[i].y );
        M->nCorner.z = std::min( M->nCorner.z, M->Vertices[i].z );
    }
    BB_pCorner.x = std::max( BB_pCorner.x, M->pCorner.x );
    BB_pCorner.y = std::max( BB_pCorner.y, M->pCorner.y );
    BB_pCorner.z = std::max( BB_pCorner.z, M->pCorner.z );
    BB_nCorner.x = std::min( BB_nCorner.x, M->nCorner.x );
    BB_nCorner.y = std::min( BB_nCorner.y, M->nCorner.y );
    BB_nCorner.z = std::min( BB_nCorner.z, M->nCorner.z );

}


/*
 * Read template mesh
 */
void load_templateMesh()
{
    template_mesh = new Mesh;
    std::cout << "Loading template mesh from " << DATA_DIR + "/Mesh_POSE/0.ply" << std::endl;
    std::string fileName = DATA_DIR + "/Mesh_POSE/0.ply";
    loadPLY( template_mesh, fileName );
    std::cout << "Loading template mesh from " << DATA_DIR + "/Mesh_POSE/0.ply ... done!"
              << std::endl << std::endl;
}


/*
 * Read instance meshes
 */
void load_instanceMeshes()
{
    BB_pCorner = glm::vec3( 0.0f, 0.0f, 0.0f );
    BB_nCorner = glm::vec3( 0.0f, 0.0f, 0.0f );

    std::string dir_POSE = DATA_DIR + "/Mesh_POSE";
    std::string dir_SHAPE = DATA_DIR + "/Mesh_SHAPE";

    numMeshes_POSE = 71;
    numMeshes_SHAPE = 37;
    // std::string meshNames_POSE[numMeshes_POSE] = {
    // 	"mesh000.ply", "mesh001.ply", "mesh002.ply", "mesh003.ply", "mesh004.ply",
    // 	"mesh005.ply", "mesh006.ply", "mesh007.ply", "mesh008.ply", "mesh009.ply",
    // 	"mesh010.ply", "mesh011.ply", "mesh012.ply", "mesh013.ply", "mesh014.ply",
    // 	"mesh015.ply", "mesh016.ply", "mesh017.ply", "mesh018.ply", "mesh019.ply",
    // 	"mesh020.ply", "mesh021.ply", "mesh022.ply", "mesh023.ply", "mesh024.ply",
    // 	"mesh025.ply", "mesh026.ply", "mesh027.ply", "mesh028.ply", "mesh029.ply",
    // 	"mesh030.ply", "mesh031.ply", "mesh032.ply", "mesh033.ply", "mesh034.ply",
    // 	"mesh035.ply", "mesh036.ply", "mesh037.ply", "mesh038.ply", "mesh039.ply",
    // 	"mesh040.ply", "mesh041.ply", "mesh042.ply", "mesh043.ply", "mesh044.ply",
    // 	"mesh045.ply", "mesh046.ply", "mesh047.ply", "mesh048.ply", "mesh049.ply",
    // 	"mesh050.ply", "mesh052.ply", "mesh053.ply", "mesh054.ply", "mesh055.ply",
    // 	"mesh056.ply", "mesh057.ply", "mesh058.ply", "mesh059.ply", "mesh060.ply",
    // 	"mesh061.ply", "mesh062.ply", "mesh063.ply", "mesh064.ply", "mesh065.ply",
    // 	"mesh066.ply", "mesh067.ply", "mesh068.ply", "mesh069.ply", "mesh070.ply",
    // 	"mesh071.ply"
    // };
    // std::string meshNames_SHAPE[numMeshes_SHAPE] = {
    // 	"SPRING0001.ply", "SPRING0002.ply", "SPRING0007.ply", "SPRING0008.ply", "SPRING0009.ply",
    // 	"SPRING0012.ply", "SPRING0013.ply", "SPRING0016.ply", "SPRING0024.ply", "SPRING0026.ply",
    // 	"SPRING0027.ply", "SPRING0028.ply", "SPRING0029.ply", "SPRING0030.ply", "SPRING0047.ply",
    // 	"SPRING0048.ply", "SPRING0049.ply", "SPRING0050.ply", "SPRING0051.ply", "SPRING0057.ply",
    // 	"SPRING0058.ply", "SPRING0062.ply", "SPRING0063.ply", "SPRING0071.ply", "SPRING0073.ply",
    // 	"SPRING0079.ply", "SPRING0080.ply", "SPRING0081.ply", "SPRING0082.ply", "SPRING0085.ply",
    // 	"SPRING0088.ply", "SPRING0098.ply", "SPRING0100.ply", "SPRING0103.ply", "SPRING0105.ply",
    // 	"SPRING0106.ply", "SPRING0107.ply"
    // };

    // numMeshes_POSE = 30;
    // numMeshes_SHAPE = 30;
    // std::string dir_POSE = DATA_DIR + "/Mesh_POSE";
    // std::string dir_SHAPE = DATA_DIR + "/Mesh_SHAPE";
    // std::string meshNames_POSE[numMeshes_POSE] = {
    // 	"file_15_15.ply", "file_15_30.ply", "file_15_45.ply", "file_15_60.ply", "file_15_75.ply", "file_15_90.ply",
    // 	"file_30_15.ply", "file_30_30.ply", "file_30_45.ply", "file_30_60.ply", "file_30_75.ply", "file_30_90.ply",
    // 	"file_45_15.ply", "file_45_30.ply", "file_45_45.ply", "file_45_60.ply", "file_45_75.ply", "file_45_90.ply",
    // 	"file_60_15.ply", "file_60_30.ply", "file_60_45.ply", "file_60_60.ply", "file_60_75.ply", "file_60_90.ply",
    // 	"file_75_15.ply", "file_75_30.ply", "file_75_45.ply", "file_75_60.ply", "file_75_75.ply", "file_75_90.ply"
    // };
    // std::string meshNames_SHAPE[numMeshes_POSE] = {
    // 		"file_15_15.ply", "file_15_30.ply", "file_15_45.ply", "file_15_60.ply", "file_15_75.ply", "file_15_90.ply",
    // 		"file_30_15.ply", "file_30_30.ply", "file_30_45.ply", "file_30_60.ply", "file_30_75.ply", "file_30_90.ply",
    // 		"file_45_15.ply", "file_45_30.ply", "file_45_45.ply", "file_45_60.ply", "file_45_75.ply", "file_45_90.ply",
    // 		"file_60_15.ply", "file_60_30.ply", "file_60_45.ply", "file_60_60.ply", "file_60_75.ply", "file_60_90.ply",
    // 		"file_75_15.ply", "file_75_30.ply", "file_75_45.ply", "file_75_60.ply", "file_75_75.ply", "file_75_90.ply"
    // };

    /*
     * This we required as initial meshes were not aligned tot he same
     * direction, so this was done to align them to the same direction
     */
    glm::mat4 Transform_POSE = glm::mat4( 1.0f );
    glm::mat4 Transform_SHAPE = glm::mat4( 1.0f );

    // if( ALIGN_MESHES_ALONG_Y_AXIS ) {
    // 	Transform_POSE = glm::rotate( glm::radians(90.0f),
    //                                   glm::vec3(0.0f,0.0f,1.0f)) * glm::rotate(glm::radians(-90.0f),
    //                                   glm::vec3(1.0f,0.0f,0.0f) );
    // 	Transform_SHAPE = glm::rotate( glm::radians(-90.0f),
    //                                    glm::vec3(1.0f,0.0f,0.0f)) * glm::rotate(glm::radians(45.0f),
    //                                    glm::vec3(0.0f,0.0f,1.0f) );
    // }

    // int meshesToRotateAlongYAxis[] = {	0,0,0,0,0,0,0,1,1,1,
    // 									1,0,1,0,0,0,0,0,0,0,
    // 									0,0,0,0,1,0,0,0,1,0,
    // 									0,0,1,0,1,0,0,0,1,0,
    // 									1,1,0,1,1,1,0,1,0,1,
    // 									1,0,1,0,1,1,1,1,0,0,
    // 									0,1,1,1,0,1,1,0,0,0,
    // 									0 };

    // Load meshes in different poses
    obj_POSE = new Mesh[numMeshes_POSE];
    std::cout << "Loading meshes from " << dir_POSE << "/[0.ply to " << numMeshes_POSE - 1
              << ".ply]" << std::endl;
    for( int m = 0; m < numMeshes_POSE; ++m ) {
        // std::string fileName = dir_POSE + "/" + meshNames_POSE[m];
        std::string fileName = dir_POSE + "/" + std::to_string( m ) + ".ply";
        loadPLY( &obj_POSE[m], fileName, Transform_POSE );
        /*if( meshesToRotateAlongYAxis[m] == 0 )
            loadPLY( &obj_POSE[m], fileName, Transform_POSE );
        else
            loadPLY( &obj_POSE[m], fileName, glm::rotate(glm::radians(180.0f),
                     glm::vec3(0.0f,1.0f,0.0f)) * Transform_POSE );*/
    }
    std::cout << "Loading meshes from " << dir_POSE << "/[0.ply to " << numMeshes_POSE - 1
              << ".ply] ... done!" << std::endl << std::endl;

    // Load meshes in different shapes
    obj_SHAPE = new Mesh[numMeshes_SHAPE];
    std::cout << "Loading meshes from " << dir_SHAPE << "/[0.ply to " << numMeshes_SHAPE - 1
              << ".ply]" << std::endl;
    for( int m = 0; m < numMeshes_SHAPE; ++m ) {
        // std::string fileName = dir_SHAPE + "/" + meshNames_SHAPE[m];
        std::string fileName = dir_SHAPE + "/" + std::to_string( m ) + ".ply";
        loadPLY( &obj_SHAPE[m], fileName, Transform_SHAPE );
    }
    std::cout << "Loading meshes from " << dir_SHAPE << "/[0.ply to " << numMeshes_SHAPE - 1
              << ".ply] ... done!" << std::endl << std::endl;

}


/*
 * Load skeketon fron file 'skeletonParts.dat'
 * and
 * Load skeketon structure from file 'joinedBones.dat'
 */
void load_skeleton()
{
    std::string skeletonParts_fileName = DATA_DIR + "/skeletonParts.dat";
    std::string joinedBones_fileName = DATA_DIR + "/joinedBones.dat";

    std::ifstream fin;

    /*
     * SOME HARDCODING
     *
     * this is not difficult to load from file,
     * but not done for no reason
     */
    if( DATA_DIR.find("data1") != std::string::npos ) {
        numBones = 16;
        numVertices = 12500;
        numFaces = 25000;
    }
    else if( DATA_DIR.find("data2") != std::string::npos ) {
        numBones = 3;
        numVertices = 20;
        numFaces = 36;
    }
    else if( DATA_DIR.find("data3") != std::string::npos ) {
        numBones = 16;
        numVertices = 6449;
        numFaces = 12894;
    }

    /*
     * Load skeleton structure
     *
     * i'th line containes 2 numbers, representing the indices of
     * two bones connected to the current bone
     */
    joinedBones.resize( numBones );

    fin.open( joinedBones_fileName );
     if( !fin.good() ) {
        ERROR( "Unable to read file " << skeletonParts_fileName );
    } // if( !fin.good() )

    int nB, a, b;
    fin >> nB;
    ASSERT( nB == numBones,
            joinedBones_fileName << " has " << nB << " bones instead of " << numBones
                << " bones" );
    for( int i = 0; i < numBones; ++i ) {
        fin >> a >> b;
        joinedBones[i] = glm::ivec2( a, b );
    }

    fin.close();

    /*
     * Load skeleton
     *
     * i'th number represents the index of bone to which i'th vertex
     * is connected
    */
    fin.open( skeletonParts_fileName );
    if( !fin.good() ) {
        ERROR( "Unable to read file " << skeletonParts_fileName );
    } // if( !fin.good() )

    int nV;
    std::cout << "Loading Skeleton data" << std::endl;
    fin >> nV >> numBones;
    ASSERT( numVertices == nV,
            skeletonParts_fileName << " has " << nV << " vertices instead of " << numVertices
                << " vertices" );

    vertBone.resize( numVertices );
    for( int i = 0; i < numVertices; ++i ) {
        fin >> vertBone[i];
    }

    // Computing faceBone from vertBone
    faceBone.resize( numFaces );
    for( size_t i = 0; i < numFaces; ++i ) {
        faceBone[i] = vertBone[ template_mesh->Faces[i].x ];
    }

    // Computing boneFaces from faceBone
    boneFaces.resize( numBones );
    for( size_t i = 0; i < numFaces; ++i ) {
        boneFaces[faceBone[i]].push_back( i );
    }

    // Initialize the color of bones
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
    auto random_generator = std::bind( distribution, generator );
    boneColor.resize( numBones );
    for( int i = 0; i < numBones; ++i ) {
        boneColor[i].x = random_generator();
        boneColor[i].y = random_generator();
        boneColor[i].z = random_generator();
    }

    std::cout << "Loading Skeleton data ... done!" << std::endl << std::endl;

    fin.close();
}


#endif //HUMAN_BODY_RECONSTRUCTION_FILE_OPERATIONS_HPP
