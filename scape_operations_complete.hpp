#ifndef HUMAN_BODY_RECONSTRUCTION_SCAPE_OPERATIONS_HPP
#define HUMAN_BODY_RECONSTRUCTION_SCAPE_OPERATIONS_HPP



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
#include "file_operations.hpp"
#include <iomanip>



/*
 ***********************************************************************
 ***********************************************************************
 *
 * Function Prototypes
 *
 ***********************************************************************
 ***********************************************************************
 */

void processData();
void load_deltaR();
void load_A();
void load_U_Mu_Sigma();

void initialize_matR( std::string filename );
void generate_deltaR();
glm::mat3 getQ_fromA( size_t face_index, double * deltaR );
std::vector< glm::mat3 > PCAspace_to_matS( arma::vec & B );
arma::vec matS_to_PCAspace( std::vector< glm::mat3 > & S_mat );

void computeFinalMesh( arma::vec & v_S );
void computeY( std::vector<glm::vec3> & RSQv1, std::vector<glm::vec3> & RSQv2 );

glm::mat3 orthogonal( glm::mat3 M );

double estimation_error_ptcld( std::vector< glm::vec3 > v_mesh );
double estimate_final_error( std::string actual_mesh_fileName );


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
 *
 */
void initData()
{
    numMeshes_POSE = 71;
    numMeshes_SHAPE = 37;

    load_templateMesh();
    load_skeleton();

    A.resize( numFaces );
    for( size_t f = 0; f < numFaces; ++f ) {
        A[f] = new double*[9];
        for( int i = 0; i < 9; ++i ) {
            A[f][i] = new double[7];
            std::fill_n( A[f][i], 7, 0 );
        }
    }

    matU.set_size( numMeshes_SHAPE, numFaces * 9 );
    vecMu.set_size( numFaces * 9 );
    vecSigma.set_size( numMeshes_SHAPE );

    Y = new Mesh;
    Y->Vertices.resize( numVertices );
    for( long int i = 0; i < numVertices; ++i ) {
        Y->Vertices[i] = glm::vec3( 0.0f, 0.0f, 0.0f );
    }
    Y->Faces.resize( numFaces );
    for( long int i = 0; i < numFaces; ++i ) {
        Y->Faces[i] = template_mesh->Faces[i];
    }
    Y->matR.resize( numBones, glm::mat3(1.0) );
    Y->matQ.resize( numFaces, glm::mat3(1.0) );
    Y->matS.resize( numFaces, glm::mat3(1.0) );
    Y->deltaR.resize( numBones );
    for( int i = 0; i < numBones; ++i ) {
        Y->deltaR[i] = new double[7];
    }
}



/*
 * Initialize mesh objects and load A and PCA parameters
 */
void load_model()
{
    // Initialize mesh objects
    initData();

    // Load A
    // It is used to compute matQ, given matR
    load_A();

    // Load PCA parameters
    // They are used to compute matS givel B
    load_U_Mu_Sigma();
}



/*
 * Load Point Cloud from PTCLD file
 */
void load_pointCloud( std::string fileName )
{
    std::cout << "Load Point Cloud" << std::endl;

    // Load file header, to be discarded
    std::ifstream fin( fileName );
    std::string file_header;
    std::getline( fin, file_header );

    // Load vertices and their correnponding vertex index
    // The file is in the form:
    //   pt.x pt.y pt.z idx
    float x, y, z;
    int i;
    while( fin >> x >> y >> z >> i ) {
        POINT_CLOUD.push_back( std::make_pair( glm::vec3(x,y,z), i ) );
    }

    std::cout << "Load Point Cloud ... done!" << std::endl << std::endl;
}



/*
 * **THIS FUNCTION IS NOT COMPLETE**
 */
void generate_mesh( std::string initial_matR_fileName ) {
    double error;

    initialize_matR(initial_matR_fileName);
    generate_deltaR();

    arma::vec v1 = arma::zeros<arma::vec>( numMeshes_SHAPE );
    computeFinalMesh( v1 );
    // std::cout << "Initial error: " << estimation_error_ptcld( template_mesh->Vertices )
    //           << std::endl;

    ERROR( "INCOMPLETE FUNCTION" );

}



/*
 * Functor for solving R required by Ceres-Solver
 * This functor represents the Cost function, which is to be minimised
 */
struct NumericDiffCostFunctor_R {
    NumericDiffCostFunctor_R( glm::vec3 & param_v_, glm::vec3 & param_v )
        : v_( param_v_ ), v( param_v ) {}

    bool operator()( const double* const r, const double* const t, double * residual ) const {
        glm::mat3 R;
        R[0] = glm::vec3( r[0], r[1], r[2] );
        R[1] = glm::vec3( r[3], r[4], r[5] );
        R[2] = glm::vec3( r[6], r[7], r[8] );

        glm::vec3 T = glm::vec3( t[0], t[1], t[2] );

        residual[0] = glm::length( ( R * v_ ) + T - v );
        return true;
    }
  private:
    const glm::vec3 v_, v;
};

/*
 * Compute R from template mesh and point cloud
 */
void initialize_matR( std::string filename )
{
    bool LOAD_INITIAL_MATR = true;

    if( LOAD_INITIAL_MATR ) {

        std::cout << "Loading initial R" << std::endl;

        std::ifstream fin( filename );
        if( !fin.good() ) {
            ERROR( "Unable to read file " + filename );
        } // if( !fin.good() )

        glm::vec3 c0, c1, c2;
        for( int b = 0; b < numBones; ++b ) {
            fin >> c0.x >> c0.y >> c0.z;
            fin >> c1.x >> c1.y >> c1.z;
            fin >> c2.x >> c2.y >> c2.z;
            Y->matR[b][0] = glm::normalize( c0 );
            Y->matR[b][1] = glm::normalize( c1 );
            Y->matR[b][2] = glm::normalize( c2 );
        }

        std::cout << "Loading matrix R ... done!" << std::endl << std::endl;

    } // if( LOAD_INITIAL_MATR )
    else {

        std::cout << "Computing initial matrix R" << std::endl;

        /*
        std::vector< std::vector< unsigned int > > bone_points( numBones );
        for( int i = 0; i < POINT_CLOUD.size(); ++i ) {
            int correspondance = POINT_CLOUD[i].second;
            int b = vertBone[correspondance];
            bone_points[ b ].push_back( i );
        }

        std::vector< double* > matR_vec( numBones ); // sizeof numBones X 9
        for( size_t i = 0; i < numBones; ++i ) {
            matR_vec[i] = new double[9];
            matR_vec[i][0] = 1;
            matR_vec[i][1] = 0;
            matR_vec[i][2] = 0;
            matR_vec[i][3] = 0;
            matR_vec[i][4] = 1;
            matR_vec[i][5] = 0;
            matR_vec[i][6] = 0;
            matR_vec[i][7] = 0;
            matR_vec[i][8] = 1;
        }
        double* vecT_vec = new double[3];

        for( size_t b = 0; b < numBones; ++b ) {

            vecT_vec[0] = 0;
            vecT_vec[1] = 0;
            vecT_vec[2] = 0;

            ceres::Problem problem;

            for( int i = 0; i < bone_points[b].size(); ++i ) {

                int correspondance = POINT_CLOUD[ bone_points[b][i] ].second;
                glm::vec3 v1_ = template_mesh->Vertices[ correspondance ];
                glm::vec3 v1 = POINT_CLOUD[ bone_points[b][i] ].first;

                problem.AddResidualBlock(
                    new ceres::NumericDiffCostFunction< NumericDiffCostFunctor_R,
                                                        ceres::CENTRAL, 1, 9, 3>(
                        new NumericDiffCostFunctor_R( v1_, v1 )
                    ),
                    NULL,
                    matR_vec[b],
                    vecT_vec
                );

            }

            ceres::Solver::Options options;
            // options.minimizer_progress_to_stdout = false;
            options.max_num_iterations = 1000;
            ceres::Solver::Summary summary;
            ceres::Solve( options, &problem, &summary );
            // std::cout << "[" << b << "]: " << summary.BriefReport() << std::endl;
        }

        for( size_t i = 0; i < numBones; ++i ) {
            Y->matR[i] = orthogonal( glm::make_mat3( matR_vec[i] ) );
        }
        */
       ERROR( "No module to compute initial matR in C++, compute it in python and load here!" )

        std::cout << "Computing initial matrix R ... done!" << std::endl << std::endl;

    } // if( LOAD_INITIAL_MATR ) {...} else
}



/*
 * Compute deltaR
 *
 * deltaR is used to get matQ from matR
 */
void generate_deltaR()
{
    std::cout << "Generating deltaR" << std::endl;

    for( size_t b = 0; b < numBones; ++b ) {
        int b1 = joinedBones[b].x;
        int b2 = joinedBones[b].y;

        // Computing deltaR - Method 2
        // using twist vector
        // done as given in the paper
        glm::mat3 R1 = Y->matR[b1] * glm::transpose( Y->matR[b] ); // glm::transpose(Y->matR[b]) * Y->matR[b1];
        glm::mat3 R2 = Y->matR[b2] * glm::transpose( Y->matR[b] ); // glm::transpose(Y->matR[b]) * Y->matR[b2];
        float trace1 = R1[0][0] + R1[1][1] + R1[2][2];
        float trace2 = R2[0][0] + R2[1][1] + R2[2][2];
        float theta1 = glm::acos( glm::clamp( ( trace1 - 1.0f ) / 2.0f,
                                              -0.99999f, 0.99999f ) );
        float theta2 = glm::acos( glm::clamp( ( trace2 - 1.0f ) / 2.0f,
                                              -0.99999f, 0.99999f ) );
        glm::vec3 t1 =
            glm::vec3( R1[1][2] - R1[2][1], R1[2][0] - R1[0][2], R1[0][1] - R1[1][0] )
            * glm::abs( theta1 ) / ( 2 * glm::sin( theta1 ) );
        glm::vec3 t2 =
            glm::vec3( R2[1][2] - R2[2][1], R2[2][0] - R2[0][2], R2[0][1] - R2[1][0] )
            * glm::abs( theta2 ) / ( 2 * glm::sin( theta2 ) );

        Y->deltaR[b] = new double[7];
        Y->deltaR[b][0] = t1.x;
        Y->deltaR[b][1] = t1.y;
        Y->deltaR[b][2] = t1.z;
        Y->deltaR[b][3] = t2.x;
        Y->deltaR[b][4] = t2.y;
        Y->deltaR[b][5] = t2.z;
        Y->deltaR[b][6] = 1;
    }

    std::cout << "Generating deltaR ... done!" << std::endl << std::endl;
}



/*
 * Load A from
 * DATA_DIR / vecA.dat
 */
void load_A()
{
    std::cout << "Loading regression vector A for all faces from vecA.dat" << std::endl;

    std::string filename = DATA_DIR + std::string( "/vecA.dat" );
    std::ifstream fin( filename );
    if( !fin.good() ) {
        ERROR( "Unable to read file " + filename );
    } // if( !fin.good() )

    for( int f = 0; f < numFaces; ++f )
        for( int j = 0; j < 3; ++j )
            for( int i = 0; i < 3; ++i )
                fin >> A[f][i * 3 + j][0] >> A[f][i * 3 + j][1] >> A[f][i * 3 + j][2]
                    >> A[f][i * 3 + j][3] >> A[f][i * 3 + j][4] >> A[f][i * 3 + j][5]
                    >> A[f][i * 3 + j][6];

    fin.close();

    std::cout << "Loading regression vector A for all faces from vecA.dat ... done!"
                << std::endl << std::endl;

}



/*
 * Load PCA parameters from
 * DATA_DIR / pca_parameters.dat
 */
void load_U_Mu_Sigma()
{
    std::cout << "Loading PCA parameters"
                << std::endl;

    std::string filename = DATA_DIR + std::string( "/pca_parameters.dat" );
    std::ifstream fin( filename );
    if( !fin.good() ) {
        ERROR( "Unable to read file " + filename );
    } // if( !fin.good() )

    for( int i = 0; i < numFaces * 9; ++i )
        fin >> vecMu[i];

    for( int i = 0; i < numMeshes_SHAPE; ++i )
        fin >> vecSigma[i];

    for( int i = 0; i < numMeshes_SHAPE; ++i )
        for( int j = 0; j < numFaces * 9; ++j )
            fin >> matU.at( i, j );

    std::cout << "Loading PCA parameters ... done!" << std::endl << std::endl;
}



/*
 * Compute matrix Q given deltaR and face_index
 */
glm::mat3 getQ_fromA( std::vector< double* > & deltaR, size_t face_index )
{
    glm::mat3 Q;

    for( int i = 0; i < 3; ++i ) {
        for( int j = 0; j < 3; ++j ) {
            Q[i][j] = A[face_index][i * 3 + j][0] * deltaR[faceBone[face_index]][0] +
                      A[face_index][i * 3 + j][1] * deltaR[faceBone[face_index]][1] +
                      A[face_index][i * 3 + j][2] * deltaR[faceBone[face_index]][2] +
                      A[face_index][i * 3 + j][3] * deltaR[faceBone[face_index]][3] +
                      A[face_index][i * 3 + j][4] * deltaR[faceBone[face_index]][4] +
                      A[face_index][i * 3 + j][5] * deltaR[faceBone[face_index]][5] +
                      A[face_index][i * 3 + j][6] * deltaR[faceBone[face_index]][6];
        }
    }

    return Q;
}



/*
 * Compute Mesh given vector B for S
 *
 * This function takes matR from template mesh,
 * matQ from detaR & A and computes matS from B
 */
void computeFinalMesh( arma::vec & B )
{
    std::vector< glm::mat3 > S_mat = PCAspace_to_matS( B );
    std::vector< glm::vec3 > RSQv1( numFaces ), RSQv2( numFaces );

    for( int f = 0; f < numFaces; ++f ) {
        glm::vec3 &F = template_mesh->Faces[f];
        glm::mat3 R = Y->matR[ faceBone[f] ];
        glm::mat3 Q = getQ_fromA( Y->deltaR, f );
        glm::mat3 S = S_mat[f];
        glm::vec3 v1 = template_mesh->Vertices[ F[1] ] - template_mesh->Vertices[ F[0] ];
        glm::vec3 v2 = template_mesh->Vertices[ F[2] ] - template_mesh->Vertices[ F[0] ];

        RSQv1[f] = R * S * Q * v1;
        RSQv2[f] = R * S * Q * v2;
    }

    computeY( RSQv1, RSQv2 );
}



/*
 * Converts matS from PCA space to original matS space
 * B = U * ( S - mu )
 */
arma::vec matS_to_PCAspace( std::vector< glm::mat3 > & S_mat )
{
    arma::vec S_mat_vectorized( numFaces * 9 );
    for( int f = 0; f < numFaces; ++f ) {
        for( int i = 0; i < 3; ++i ) {
            for( int j = 0; j < 3; ++j ) {
                S_mat_vectorized[ f * 9 + i * 3 + j ] = S_mat[f][j][i];
            }
        }
    }

    return matU * ( S_mat_vectorized - vecMu );
}

/*
 * Converts matS from original matS space to PCA space
 * S = ( U.T * B ) + mu
 */
std::vector< glm::mat3 > PCAspace_to_matS( arma::vec & B )
{
    arma::vec S_mat_vectorized = ( matU.t() * B ) + vecMu;

    std::vector< glm::mat3 > S_mat( numFaces, glm::mat3( 1.0 ) );
    for( int f = 0; f < numFaces; ++f ) {
        for( int i = 0; i < 3; ++i ) {
            for( int j = 0; j < 3; ++j ) {
                S_mat[f][j][i] = float( S_mat_vectorized[ f * 9 + i * 3 + j ] );
            }
        }
    }

    return S_mat;
}



/*
 * Functor for solving Y required by Ceres-Solver
 * This functor represents the Cost function, which is to be minimised
 */
struct AutoDiffCostFunctor_Y {
    AutoDiffCostFunctor_Y( double param_RSQv_x )
        : RSQv_x( param_RSQv_x ) {}

    template <typename T>
    bool operator()( const T* const Y_kj_x, const T* const Y_k1_x, T* residual ) const
    {
        residual[0] = RSQv_x - ( Y_kj_x[0] - Y_k1_x[0] );
        return true;
    }
  private:
    const double RSQv_x;
};

/*
 * Compute Mesh given all the face vectors
 */
void computeY( std::vector< glm::vec3 > & RSQv1, std::vector< glm::vec3 > & RSQv2 )
{
    /*
    * E[Y] = sum{   sum{    R * S * Q * Vk - (Yk,j - Yk,1)    }    }
    *         k     j=2,3  \______  ______/
    *                             \/
    *                            RSQv
    *
    * where
    * E[Y] = points to be estimated
    * R = 3x3 Rotation matrix
    * Q = 3x3 Transformation matrix, computed using A and deltaR
    * S = 3x3 Transformation matrix, computed using vecB and PCA params
    * RSQv1 = 3x1 vector, precomputed, with j = 1
    * RSQv2 = 3x1 vector, precomputed, with j = 2
    */
    double* y[3];
    for( int i = 0; i < 3; ++i ) {
        y[i] = new double[numVertices];
    }
    for( int i = 0; i < numVertices; ++i ) {
        y[0][i] = 0.0f;
        y[1][i] = 0.0f;
        y[2][i] = 0.0f;
    }

    std::cout << "Computing Final Mesh" << std::endl;

    for( int i = 0; i < 3; ++i ) {
        // i = 0  ->  Y_x
        // i = 1  ->  Y_y
        // i = 2  ->  Y_z

        ceres::Problem problem;

        for( int f = 0; f < numFaces; ++f ) {

            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction< AutoDiffCostFunctor_Y, 1, 1, 1 >(
                    new AutoDiffCostFunctor_Y( RSQv1[f][i] )
                ),
                NULL,
                &y[i][ size_t( template_mesh->Faces[f].y ) ],
                &y[i][ size_t( template_mesh->Faces[f].x ) ]
            );

            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction< AutoDiffCostFunctor_Y, 1, 1, 1 >(
                    new AutoDiffCostFunctor_Y( RSQv2[f][i] )
                ),
                NULL,
                y[i] + size_t( template_mesh->Faces[f].z ),
                y[i] + size_t( template_mesh->Faces[f].x )
            );

        }

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve( options, &problem, &summary );
        // std::cout << summary.BriefReport() << std::endl << std::endl;
    }

    std::cout << "Computing Final Mesh ... done!" << std::endl << std::endl;

    for( int v = 0; v < numVertices; ++v ) {
        Y->Vertices[v] = glm::vec3( y[0][v], y[1][v], y[2][v] );
    }

    Y->pCorner.x = Y->Vertices[0].x;
    Y->pCorner.y = Y->Vertices[0].y;
    Y->pCorner.z = Y->Vertices[0].z;
    Y->nCorner.x = Y->Vertices[0].x;
    Y->nCorner.y = Y->Vertices[0].y;
    Y->nCorner.z = Y->Vertices[0].z;
    for( int i = 1; i < numVertices; ++i ) {
        Y->pCorner.x = std::max( Y->pCorner.x, Y->Vertices[i].x );
        Y->pCorner.y = std::max( Y->pCorner.y, Y->Vertices[i].y );
        Y->pCorner.z = std::max( Y->pCorner.z, Y->Vertices[i].z );
        Y->nCorner.x = std::min( Y->nCorner.x, Y->Vertices[i].x );
        Y->nCorner.y = std::min( Y->nCorner.y, Y->Vertices[i].y );
        Y->nCorner.z = std::min( Y->nCorner.z, Y->Vertices[i].z );
    }
    BB_pCorner.x = std::max( BB_pCorner.x, Y->pCorner.x );
    BB_pCorner.y = std::max( BB_pCorner.y, Y->pCorner.y );
    BB_pCorner.z = std::max( BB_pCorner.z, Y->pCorner.z );
    BB_nCorner.x = std::min( BB_nCorner.x, Y->nCorner.x );
    BB_nCorner.y = std::min( BB_nCorner.y, Y->nCorner.y );
    BB_nCorner.z = std::min( BB_nCorner.z, Y->nCorner.z );
}



/*
 * Test estimated mesh in Y with the point cloud
 *
 * Ground truth : i'th vertex posiiton in point cloud
 * Estimated position : corresponding vertex position in estimated mesh
 *
 * Error[i] = abs( Ground_truth - Estimated_position )
 * Avg Error = avg( Error[i] ), for all i in Vertices
 *
 * Percentage Error = Avg Error / Height of template mesh
 */
double estimation_error_ptcld( std::vector< glm::vec3 > v_mesh )
{
    double error = 0.0;
    for( int i = 0; i < POINT_CLOUD.size(); ++i ) {
        glm::vec3 gt_i = POINT_CLOUD[i].first;
        int correspondance = POINT_CLOUD[i].second;
        glm::vec3 estimate_point_i = v_mesh[ correspondance ];

        error += glm::length( gt_i - estimate_point_i );
    }

    error /= POINT_CLOUD.size();
    double height_template_mesh = template_mesh->pCorner.y - template_mesh->nCorner.y;

    return error / height_template_mesh;
}


double estimate_final_error( std::string actual_mesh_fileName )
{
    ERROR( "NOT IMPLEMENTED YET" );
    return 0.0;
}


#endif //HUMAN_BODY_RECONSTRUCTION_SCAPE_OPERATIONS_HPP
