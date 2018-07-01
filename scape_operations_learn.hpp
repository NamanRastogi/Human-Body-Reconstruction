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
#include <iomanip> // setprecision



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
void computeR();
void computeQ();
void computeS();
void compute_deltaR();
void computeA();
void computeU_Mu_Sigma();
void computeFinalMesh( int m_P, int m_S );
void computeFinalMesh( int m_P, arma::vec & B );
void computeY( std::vector<glm::vec3> & RSQv1, std::vector<glm::vec3> & RSQv2 );
glm::mat3 getQ_fromA( size_t face_index, double * deltaR );
void testQ();
void testS();
void testU_Mu_Sigma();




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
 * Load meshes & skeleton and initialize mesh objects
 */
void initData()
{
    // Load template mesh from location :
    //     "DATA_DIR / Meshes_POSE / 0.ply"
    load_templateMesh();

    // Load all meshes from locations :
    //     "DATA_DIR / Meshes_POSE / [0-70].ply"
    //     "DATA_DIR / Meshes_SHAPE / [0-36].ply"
    load_instanceMeshes();

    // Load skeketon from location :
    //     "DATA_DIR / skeletonParts.dat"
    load_skeleton();

    // Print nerdy stuff about meshes
    std::cout << "Stats about meshes: " << std::endl
              << "\tMeshes in Pose Space : " << numMeshes_POSE << std::endl
              << "\tMeshes in Shape Space : " << numMeshes_SHAPE << std::endl
              << "\tBones per Mesh : " << numBones << std::endl
              << "\tVertices per Mesh : " << numVertices << std::endl
              << "\tFaces per Mesh : " << numFaces << std::endl << std::endl;

    /*
     * Initialize mesh objects
     *
     * Keep in mind that "Faces", "Vertices" (and "Normals" if
     * applicable) of obj_POSE[0-70] and obj_SHAPE[0-36] have already
     * been initialized above while loading them
     */
    for( size_t m = 0; m < numMeshes_POSE; ++m ) {
        // obj_POSE[m].boneDirection.resize( numBones );
        obj_POSE[m].matR.resize( numBones, glm::mat3( 1.0 ) );
        obj_POSE[m].matQ.resize( numFaces, glm::mat3( 1.0 ) );
        obj_POSE[m].matS.resize( numFaces, glm::mat3( 1.0 ) );
        obj_POSE[m].deltaR.resize( numBones );
        for( int b = 0; b < numBones; ++b )
            obj_POSE[m].deltaR[b] = new double[7];
    }

    for( size_t m = 0; m < numMeshes_SHAPE; ++m ) {
        // obj_SHAPE[m].boneDirection.resize( numBones );
        obj_SHAPE[m].matR.resize( numBones, glm::mat3( 1.0 ) );
        obj_SHAPE[m].matQ.resize( numFaces, glm::mat3( 1.0 ) );
        obj_SHAPE[m].matS.resize( numFaces, glm::mat3( 1.0 ) );
        obj_SHAPE[m].deltaR.resize( numBones );
        for( int b = 0; b < numBones; ++b )
            obj_SHAPE[m].deltaR[b] = new double[7];
    }

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

    // Mesh to store final mesh
    Y = new Mesh;
    Y->Faces.resize( numFaces );
    for( long int i = 0; i < numFaces; ++i ) {
        Y->Faces[i] = template_mesh->Faces[i];
    }
    Y->Vertices.resize( numVertices );

}


/*
 * Compute matR, matQ, matS, deltaR, regression vector A
 * and PCA parameters
 */
void processData()
{
    // Load or Compute Rotation matrices R
    computeR();

    // Load or Compute Pose-Space Deformation matrices Q and test them
    computeQ();
    testQ();

    // Load or Compute Shape-Space Deformation matrices S and test them
    computeS();
    testS();

    // Compute deltaR
    // It is computed using twist vector, check paper for more details
    compute_deltaR();

    // Load or Compute regression vector A
    // It is used to generate matQ from matR
    computeA();

    // Load or Compute PCA parameters
    // It is used to generate matS from vector B
    computeU_Mu_Sigma();

    // arma::vec b1 = arma::zeros<arma::vec>( numMeshes_SHAPE );
    // b1[0] -= 0.2 * vecSigma[0];
    // computeFinalMesh( 7, b1 );

    // std::string filename = DATA_DIR + "/Y4.ply";
    // savePLY( Y, filename );
    // std::cout << "Writing " << filename << " on the disk ... done!" << std::endl << std::endl;
}


/*
 * Load or Compute Rotation matrices R
 */
void computeR()
{
    /*
     * Initialize matR of Template mesh
     */
    template_mesh->matR.resize( numBones, glm::mat3(1.0) );

    /*
     * Initialize matR of obj_POSE meshes
     */
    if( LOAD_R_FROM_DISK ) {
        // Load matR from disk

        std::cout << "Loading matrices R from matR/[0.dat to " << numMeshes_POSE - 1 << ".dat]"
                  << std::endl;

        for( int m = 0; m < numMeshes_POSE; ++m ) {
            std::string filename = DATA_DIR + std::string( "/matR/" ) + std::to_string( m )
                                   + std::string( ".dat" );
            std::ifstream fin( filename );
            if( !fin.good() ) {
                ERROR( "Unable to read file " + filename );
            } // if( !fin.good() )

            glm::vec3 c0, c1, c2;
            for( int b = 0; b < numBones; ++b ) {
                fin >> c0.x >> c0.y >> c0.z;
                fin >> c1.x >> c1.y >> c1.z;
                fin >> c2.x >> c2.y >> c2.z;
                obj_POSE[m].matR[b][0] = glm::normalize( c0 );
                obj_POSE[m].matR[b][1] = glm::normalize( c1 );
                obj_POSE[m].matR[b][2] = glm::normalize( c2 );
            }

        }

        std::cout << "Loading matrices R from matR/[0.dat to " << numMeshes_POSE - 1
                  << ".dat] ... done!" << std::endl << std::endl;

    } // if( LOAD_R_FROM_DISK )
    else {
        // Compute matR
        // Earlier it was done using Joine Locations and PCA, it is
        // deprecated now. Use ICP to do it.
        // It is done in python, but not here in C++

        ERROR( "Computing matrices R using boneDirection is deprecated, compute using ICP "
                 "instead!" );

        if( SAVE_R_TO_DISK ) {
            std::ofstream fout;

            std::cout << "\tWriting files " << DATA_DIR + std::string( "/matR/[0.dat to " )
                      << numMeshes_POSE - 1 << ".dat]";
            for( int m = 0; m < numMeshes_POSE; ++m ) {
                std::string filename = DATA_DIR + std::string( "/matR/" ) + std::to_string( m )
                                       + std::string( ".dat" );
                fout.open( filename, std::ios::trunc );
                for( int i = 0; i < obj_POSE[m].matR.size(); ++i ) {
                    fout << obj_POSE[m].matR[i][0][0] << " " << obj_POSE[m].matR[i][0][1] << " "
                         << obj_POSE[m].matR[i][0][2] << " " << obj_POSE[m].matR[i][1][0] << " "
                         << obj_POSE[m].matR[i][1][1] << " " << obj_POSE[m].matR[i][1][2] << " "
                         << obj_POSE[m].matR[i][2][0] << " " << obj_POSE[m].matR[i][2][1] << " "
                         << obj_POSE[m].matR[i][2][2] << std::endl;
                }
                fout.close();
            }
            std::cout << "\tWriting files " << DATA_DIR + std::string( "/matR/[0.dat to " )
                      << numMeshes_POSE - 1 << ".dat] ... done!" << std::endl;
        } // if( SAVE_R_TO_DISK )

    } // if( LOAD_R_FROM_DISK ) {...} else
}



/*
 * Functor for solving Q required by Ceres-Solver
 * This functor represents the Cost function, which is to be minimised
 */
struct NumericDiffCostFunctor_Q1 {
    NumericDiffCostFunctor_Q1( glm::mat3 & param_R, glm::vec3 & param_v_, glm::vec3 & param_v )
        : R( param_R ), v_( param_v_ ), v( param_v ) {}

    bool operator()( const double * const q, double * residual ) const {
        glm::mat3 Q;
        Q[0] = glm::vec3( q[0], q[1], q[2] );
        Q[1] = glm::vec3( q[3], q[4], q[5] );
        Q[2] = glm::vec3( q[6], q[7], q[8] );
        residual[0] = glm::length( ( R * Q * v_ ) - v );
        return true;
    }
  private:
    const glm::mat3 R;
    const glm::vec3 v_, v;
};

/*
 * Functor for solving Q (regularizer) required by Ceres-Solver
 * This functor represents the Cost function, which is to be minimised
 */
struct NumericDiffCostFunctor_Q2 {
    NumericDiffCostFunctor_Q2( double param_rho )
        : rho( param_rho ) {}

    bool operator()( const double * const q1, const double * const q2, double * residual ) const
    {
        residual[0] = 0.0001 * rho * matDiff_forbNorm( q1, q2 );
        return true;
    }
  private:
    const double rho;

    inline double matDiff_forbNorm( const double * const m1, const double * const m2 ) const
    {
        double sum = 0.0;
        for( size_t i = 0; i < 9; ++i )
            sum += std::pow( m1[i] - m2[i], 2 );
        return std::sqrt( sum );
    }
};

/*
 * Load or Compute Pose-Space Deformation matrices Q
 */
void computeQ()
{
    /*
     * Initialize matQ of Template Mesh
     */
    template_mesh->matQ.resize( numFaces, glm::mat3(1.0) );

    /*
     * Initialize matQ of obj_POSE meshes
     */
    if( LOAD_Q_FROM_DISK ) {
        // Load matQ from disk

        std::cout << "Loading matrices Q from matQ/[0.dat to " << numMeshes_POSE - 1 << ".dat]"
                  << std::endl;

        for( int m = 0; m < numMeshes_POSE; ++m ) {

            std::string filename_m = DATA_DIR + std::string( "/matQ/" ) + std::to_string( m )
                                     + std::string( ".dat" );
            std::ifstream fin( filename_m );
            if( !fin.good() ) {
                ERROR( "Unable to read file " + filename_m );
            } // if( !fin.good() )

            // std::cout << "Loading " << filename_m << std::endl;

            for( int f = 0; f < numFaces; ++f ) {
                double q[9];
                fin >> q[0] >> q[1] >> q[2];
                fin >> q[3] >> q[4] >> q[5];
                fin >> q[6] >> q[7] >> q[8];
                obj_POSE[m].matQ[f][0] = glm::vec3( q[0], q[1], q[2] );
                obj_POSE[m].matQ[f][1] = glm::vec3( q[3], q[4], q[5] );
                obj_POSE[m].matQ[f][2] = glm::vec3( q[6], q[7], q[8] );
            }

            fin.close();

        }

        std::cout << "Loading matrices Q from matQ/[0.dat to " << numMeshes_POSE - 1
                  << ".dat] ... done!" << std::endl << std::endl;

    } // if( LOAD_Q_FROM_DISK )
    else {
        // Compute matQ

        std::cout << "Computing matrices Q from matQ/[0 to " << numMeshes_POSE - 1 << "]"
                  << std::endl;

        for( size_t m = 0; m < numMeshes_POSE; ++m ) {

            // matQ_vec is the variable for Ceres-Solver
            std::vector<double *> matQ_vec( numFaces ); // sizeof numFaces X 9
            for( size_t i = 0; i < numFaces; ++i ) {
                matQ_vec[i] = new double[9];
                matQ_vec[i][0] = 1;
                matQ_vec[i][1] = 0;
                matQ_vec[i][2] = 0;
                matQ_vec[i][3] = 0;
                matQ_vec[i][4] = 1;
                matQ_vec[i][5] = 0;
                matQ_vec[i][6] = 0;
                matQ_vec[i][7] = 0;
                matQ_vec[i][8] = 1;
            }

            for( size_t b = 0; b < numBones; ++b ) {

                std::cout << std::endl
                          << "Processing Mesh " << m << " ( Bone " << b + 1 << " of " << numBones
                          << " ) : total " << boneFaces[b].size() << " vertices." << std::endl;

                // Initialize the ceres problem
                ceres::Problem problem;

                for( int f = 0; f < boneFaces[b].size(); ++f ) {

                    int fi = boneFaces[b][f];
                    glm::vec3 &F  = template_mesh->Faces[ fi ];


                    // R * Q * v2_ = v2
                    glm::vec3 v1_ =
                        template_mesh->Vertices[ F[1] ] - template_mesh->Vertices[ F[0] ];
                    glm::vec3 v2_ =
                        template_mesh->Vertices[ F[2] ] - template_mesh->Vertices[ F[0] ];
                    glm::vec3 v1  = obj_POSE[m].Vertices[ F[1] ] - obj_POSE[m].Vertices[ F[0] ];
                    glm::vec3 v2  = obj_POSE[m].Vertices[ F[2] ] - obj_POSE[m].Vertices[ F[0] ];

                    // Add
                    // R * Q * v1_ = v1
                    // to the problem
                    problem.AddResidualBlock(
                        new ceres::NumericDiffCostFunction< NumericDiffCostFunctor_Q1,
                                                            ceres::CENTRAL, 1, 9 >(
                            new NumericDiffCostFunctor_Q1( obj_POSE[m].matR[b], v1_, v1 )
                        ),
                        NULL,
                        matQ_vec[fi]
                    );

                    // Add
                    // R * Q * v1_ = v1
                    // to the problem
                    problem.AddResidualBlock(
                        new ceres::NumericDiffCostFunction< NumericDiffCostFunctor_Q1,
                                                            ceres::CENTRAL, 1, 9 >(
                            new NumericDiffCostFunctor_Q1( obj_POSE[m].matR[b], v2_, v2 )
                        ),
                        NULL,
                        matQ_vec[fi]
                    );

                }

                // Regularizer
                for( int f1 = 0; f1 < boneFaces[b].size(); ++f1 ) {
                    for( int f2 = f1 + 1; f2 < boneFaces[b].size(); ++f2 ) {

                        int f1i = boneFaces[b][f1];
                        int f2i = boneFaces[b][f2];

                        problem.AddResidualBlock(
                            new ceres::NumericDiffCostFunction< NumericDiffCostFunctor_Q2,
                                                                ceres::CENTRAL, 1, 9, 9>(
                                new NumericDiffCostFunctor_Q2( 0.001 )
                            ),
                            NULL,
                            matQ_vec[f1i],
                            matQ_vec[f2i]
                        );

                    }
                }

                ceres::Solver::Options options;
                // options.minimizer_progress_to_stdout = false;
                // options.max_num_iterations = 50;
                ceres::Solver::Summary summary;
                ceres::Solve( options, &problem, &summary );
                std::cout << summary.BriefReport() << std::endl;
            }

            for( size_t i = 0; i < numFaces; ++i ) {
                obj_POSE[m].matQ[i] = glm::make_mat3( matQ_vec[i] );
            }

            if( SAVE_Q_TO_DISK ) {
                std::cout << "\tWriting files " << DATA_DIR + std::string( "/matQ/[0.dat to " )
                          << numMeshes_POSE - 1 << ".dat]" << std::endl;
                std::string filename_m = DATA_DIR + std::string( "/matQ/" ) + std::to_string( m )
                                         + std::string( ".txt" );
                std::cout << "Writing file " << filename_m << std::endl;
                std::ofstream fout_Q( filename_m, std::ios::trunc );
                for( int i = 0; i < numFaces; ++i ) {
                    for( int j = 0; j < 9; ++j ) {
                        fout_Q << matQ_vec[i][j] << " ";
                    }
                    fout_Q << "\n";
                }
                std::cout << "\tWriting files " << DATA_DIR + std::string( "/matQ/[0.dat to " )
                          << numMeshes_POSE - 1 << ".dat] ... done!" << std::endl;
            } // if( SAVE_Q_TO_DISK )

        }

        std::cout << "Computing matrices Q from matQ/[0 to " << numMeshes_POSE - 1 << "] ... done!"
                  << std::endl << std::endl;

    } // if( LOAD_Q_FROM_DISK ) {...} else
}



/*
 * Load or Compute Shape-Space Deformation matrices S
 */
void computeS()
{
    /*
     * Initialize matS of Template Mesh
     */
    template_mesh->matS.resize( numFaces, glm::mat3(1.0) );

    /*
     * Initialize matS of obj_SHAPE meshes
     */
    if( LOAD_S_FROM_DISK ) {

        std::cout << "Loading matrices S from matS/[0.dat to " << numMeshes_SHAPE - 1 << ".dat]"
                  << std::endl;

        for( int m = 0; m < numMeshes_SHAPE; ++m ) {

            std::string filename_m = DATA_DIR + std::string( "/matS/" ) + std::to_string( m )
                                     + std::string( ".dat" );
            std::ifstream fin( filename_m );
            if( !fin.good() ) {
                ERROR( "Unable to read file " + filename_m );
            } // if( !fin.good() )

            for( int f = 0; f < numFaces; ++f ) {
                double s[9];
                fin >> s[0] >> s[1] >> s[2];
                fin >> s[3] >> s[4] >> s[5];
                fin >> s[6] >> s[7] >> s[8];
                obj_SHAPE[m].matS[f][0] = glm::vec3( s[0], s[1], s[2] );
                obj_SHAPE[m].matS[f][1] = glm::vec3( s[3], s[4], s[5] );
                obj_SHAPE[m].matS[f][2] = glm::vec3( s[6], s[7], s[8] );
            }

            fin.close();

        }

        std::cout << "Loading matrices S from matS/[0.dat to " << numMeshes_POSE - 1
                  << ".dat] ... done!" << std::endl << std::endl;

    } // if( LOAD_S_FROM_DISK )
    else {
        std::cerr << "NO MODULE TO COMPUTE matS" << std::endl;
        std::exit( EXIT_FAILURE );
    }
}


/*
 * Compute deltaR
 *
 * deltaR and vecA together are used to get matQ from matR
 */
void compute_deltaR()
{
    std::cout << "Computing deltaR" << std::endl;

    for( size_t m = 0; m < numMeshes_POSE; ++m ) {
        for( size_t b = 0; b < numBones; ++b ) {
            int b1 = joinedBones[b].x;
            int b2 = joinedBones[b].y;

            // Computing deltaR - Method 1
            // computation using boneDirection is deprecated
            // boneDirection can be computed using
            // jointLocations or PCA

            // Computing deltaR - Method 2
            // using twist vector
            // done as given in the paper
            glm::mat3 R1 = obj_POSE[m].matR[b1] * glm::transpose( obj_POSE[m].matR[b] ); // glm::transpose(obj_POSE[m].matR[b]) * obj_POSE[m].matR[b1];
            glm::mat3 R2 = obj_POSE[m].matR[b2] * glm::transpose( obj_POSE[m].matR[b] ); // glm::transpose(obj_POSE[m].matR[b]) * obj_POSE[m].matR[b2];
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

            // Computing deltaR - Method 3
            // using twist vector
            // done using quaternion
            // glm::mat3 R1 = obj_POSE[m].matR[b1] * glm::transpose(obj_POSE[m].matR[b]); // glm::transpose(obj_POSE[m].matR[b]) * obj_POSE[m].matR[b1];
            // glm::mat3 R2 = obj_POSE[m].matR[b2] * glm::transpose(obj_POSE[m].matR[b]); // glm::transpose(obj_POSE[m].matR[b]) * obj_POSE[m].matR[b2];
            // glm::quat q1 = glm::quat_cast( R1 );
            // glm::quat q2 = glm::quat_cast( R2 );
            // glm::vec3 t1 = glm::axis(q1) * glm::angle(q1);
            // glm::vec3 t2 = glm::axis(q2) * glm::angle(q2);

            obj_POSE[m].deltaR[b][0] = t1.x;
            obj_POSE[m].deltaR[b][1] = t1.y;
            obj_POSE[m].deltaR[b][2] = t1.z;
            obj_POSE[m].deltaR[b][3] = t2.x;
            obj_POSE[m].deltaR[b][4] = t2.y;
            obj_POSE[m].deltaR[b][5] = t2.z;
            obj_POSE[m].deltaR[b][6] = 1;
        }
    }

    std::cout << "Computing deltaR ... done!" << std::endl << std::endl;
}

/*
 * Functor for solving A required by Ceres-Solver
 * This functor represents the Cost function, which is to be minimised
 */
struct AutoDiffCostFunctor_A {
    AutoDiffCostFunctor_A( double * param_deltaR, double param_q_lm )
        : deltaR( param_deltaR ), q_lm( param_q_lm ) {}

    template <typename T>
    bool operator()( const T* const A, T* residual ) const
    {
        residual[0] = ( deltaR[0] * A[0] ) + ( deltaR[1] * A[1] ) + ( deltaR[2] * A[2] ) +
                      ( deltaR[3] * A[3] ) + ( deltaR[4] * A[4] ) + ( deltaR[5] * A[5] ) +
                      ( deltaR[6] * A[6] ) - q_lm;
        return true;
    }
  private:
    const double * deltaR;
    const double q_lm;
};

/*
 * Load or Compute
 * deltaR and vecA together are used to generate matQ from matR
 */
void computeA()
{
    if( LOAD_A_FROM_DISK ) {
        // Load matQ from disk

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

    } // if( LOAD_A_FROM_DISK )
    else {
        // Compute matQ

        std::cout << "Computing regression vector A for all faces" << std::endl;

        // A is the variable for Ceres-Solver

        for( size_t f = 0; f < numFaces; ++f ) {

            // Progress Bar
            std::cout.flush();
            std::cout << "\r" << std::setprecision(4) << float(f) / numFaces * 100 << "\% ";

            for( int i = 0; i < 3; ++i ) {
                for( int j = 0; j < 3; ++j ) {

                    // Initialize the ceres problem
                    ceres::Problem problem;

                    for( size_t m = 0; m < numMeshes_POSE; ++m ) {

                        // Add
                        // A[f] * deltaR[faceBone[f]] = matQ[f]
                        // to the problem
                        problem.AddResidualBlock(
                            new ceres::AutoDiffCostFunction<AutoDiffCostFunctor_A, 1, 7>(
                                new AutoDiffCostFunctor_A( obj_POSE[m].deltaR[faceBone[f]],
                                                           obj_POSE[m].matQ[f][i][j] )
                            ),
                            NULL,
                            A[f][i * 3 + j]
                        );

                    }

                    ceres::Solver::Options options;
                    options.minimizer_progress_to_stdout = false;
                    options.max_num_iterations = 1000;
                    ceres::Solver::Summary summary;
                    ceres::Solve( options, &problem, &summary );
                    // std::cout << summary.BriefReport() << std::endl;

                }
            }

        }

        std::cout << "\rComputing regression vector A for all faces ... done!    "
                  << std::endl << std::endl;

        if( SAVE_A_TO_DISK ) {
            std::cout << "Saving regression vector A for all faces to vecA.dat" << std::endl;

            std::string filename = DATA_DIR + std::string( "/vecA.dat" );
            std::ofstream fout( filename, std::ios::trunc );

            for( int f = 0; f < numFaces; ++f ) {
                for( int j = 0; j < 3; ++j )
                    for( int i = 0; i < 3; ++i )
                        fout << A[f][i * 3 + j][0] << " " << A[f][i * 3 + j][1] << " "
                             << A[f][i * 3 + j][2] << " " << A[f][i * 3 + j][3] << " "
                             << A[f][i * 3 + j][4] << " " << A[f][i * 3 + j][5] << " "
                             << A[f][i * 3 + j][6] << " ";
                fout << std::endl;
            }
        } // if( SAVE_A_TO_DISK )

        std::cout << "Saving regression vector A for all faces to vecA.dat ... done!" << std::endl
                  << std::endl;

    }  // if( LOAD_A_FROM_DISK ) {...} else

}



/*
 * Load or Compute PCA parameters from disk
 */
void computeU_Mu_Sigma()
{
    if( LOAD_U_Mu_Sigma_FROM_DISK ) {

        std::cout << "Loading PCA parameters" << std::endl;

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

    } // if( LOAD_A_FROM_DISK )
    else {

        std::cout << "Computing PCA parameters U, Mu and Sigma (" << numFaces << ")" << std::endl;

        ERROR( "Difficult to compute in CPP, compute in Python instead using "
               "sklearn.decomposition.PCA." );

        std::cout << "Computing PCA parameters U, Mu and Sigma (" << numFaces << ") ... done!"
                  << std::endl << std::endl;
    }
}



/*
 * Compute matrix Q given deltaR and face_index
 */
glm::mat3 getQ_fromA( std::vector< double* > & deltaR, size_t face_index )
{
    glm::mat3 Q;

    for( int i = 0; i < 3; ++i ) {
        for( int j = 0; j < 3; ++j ) {
            Q[i][j] = A[face_index][i * 3 + j][0] * deltaR[faceBone[face_index]][0]
                      + A[face_index][i * 3 + j][1] * deltaR[faceBone[face_index]][1]
                      + A[face_index][i * 3 + j][2] * deltaR[faceBone[face_index]][2]
                      + A[face_index][i * 3 + j][3] * deltaR[faceBone[face_index]][3]
                      + A[face_index][i * 3 + j][4] * deltaR[faceBone[face_index]][4]
                      + A[face_index][i * 3 + j][5] * deltaR[faceBone[face_index]][5]
                      + A[face_index][i * 3 + j][6] * deltaR[faceBone[face_index]][6];
        }
    }

    return Q;
}



/*
 * Compute Mesh given indices of Pose and Shape mesh.
 *
 * This function takes matR and matQ from obj_POSE[m_P] and
 * matS from obj_SHAPE[m_S]
 */
void computeFinalMesh( int m_P, int m_S )
{
    std::vector< glm::vec3 > RSQv1( numFaces ), RSQv2( numFaces );

    for( int f = 0; f < numFaces; ++f ) {
        glm::vec3 &F = template_mesh->Faces[f];
        glm::mat3 R = obj_POSE[m_P].matR[ faceBone[f] ];
        glm::mat3 Q = obj_POSE[m_P].matQ[f];
        glm::mat3 S = obj_SHAPE[m_S].matS[f];
        glm::vec3 v1 = template_mesh->Vertices[F[1]] - template_mesh->Vertices[F[0]];
        glm::vec3 v2 = template_mesh->Vertices[F[2]] - template_mesh->Vertices[F[0]];

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
std::vector< glm::mat3 > PCAspace_to_matS( arma::vec & S_pca )
{
    arma::vec S_mat_vectorized = ( matU.t() * S_pca ) + vecMu;

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
 * Compute Mesh given index of Pose and B vector for S
 *
 * This function takes matR and matQ from obj_POSE[m_P] and
 * computes matS from B
 */
void computeFinalMesh( int m_P, arma::vec & B )
{
    std::vector< glm::vec3 > RSQv1( numFaces ), RSQv2( numFaces );

    std::vector< glm::mat3 > S_mat = PCAspace_to_matS( B );

    for( int f = 0; f < numFaces; ++f ) {
        glm::vec3 &F = template_mesh->Faces[f];
        glm::mat3 R = obj_POSE[m_P].matR[ faceBone[f] ];
        glm::mat3 Q = getQ_fromA( obj_POSE[m_P].deltaR, f ); // obj_POSE[m_P].matQ[f];
        glm::mat3 S = S_mat[f];
        glm::vec3 v1 = template_mesh->Vertices[F[1]] - template_mesh->Vertices[F[0]];
        glm::vec3 v2 = template_mesh->Vertices[F[2]] - template_mesh->Vertices[F[0]];

        RSQv1[f] = R * S * Q * v1;
        RSQv2[f] = R * S * Q * v2;
    }

    computeY( RSQv1, RSQv2 );
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
    double * y[3];
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

    // Update bounding box
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
 * Test matrices Q
 *
 * Ground truth : i'th vertex posiiton in k'th mesh
 * Template position : i'th vertex posiiton in template mesh
 * Estimated position : R * Q * Template_position
 *
 * Error[i] = abs( Ground_truth - Estimated_position )
 * Avg Error = avg( Error[i] ), for all i in Vertices
 *
 * Percentage Error = Avg Error / Height of template mesh
 */
void testQ()
{
    std::cout << "Testing Q :" << std::endl;

    float overall_avg_error = 0.0;

    for( size_t m = 0; m < numMeshes_POSE; ++m ) {
        std::vector<glm::mat3> &matR = obj_POSE[m].matR;
        std::vector<double *> &deltaR = obj_POSE[m].deltaR;

        float max_error = 0.0;
        float avg_error = 0.0;

        for( int f = 0; f < numFaces; ++f ) {
            glm::vec3 &F = template_mesh->Faces[f];
            glm::vec3 v1_ = template_mesh->Vertices[ F[1] ] - template_mesh->Vertices[ F[0] ];
            glm::vec3 v2_ = template_mesh->Vertices[ F[2] ] - template_mesh->Vertices[ F[0] ];
            glm::vec3 v1_real = obj_POSE[m].Vertices[ F[1] ] - obj_POSE[m].Vertices[ F[0] ];
            glm::vec3 v2_real = obj_POSE[m].Vertices[ F[2] ] - obj_POSE[m].Vertices[ F[0] ];
            glm::vec3 v1_computed = obj_POSE[m].matR[faceBone[f]] * obj_POSE[m].matQ[f] * v1_;
            glm::vec3 v2_computed = obj_POSE[m].matR[faceBone[f]] * obj_POSE[m].matQ[f] * v2_;

            glm::vec3 v1_diff = v1_computed - v1_real;
            glm::vec3 v2_diff = v2_computed - v2_real;

            max_error = std::max( max_error, glm::length( v1_diff ) );
            max_error = std::max( max_error, glm::length( v2_diff ) );
            avg_error += glm::length( v1_diff );
            avg_error += glm::length( v2_diff );
        }

        avg_error /= ( numFaces * 2 );
        // std::cout << "\t[ Mesh " << m << " ]\t\tMax Error: " << max_error << "\t\tAvg Error: "
        //           << avg_error << std::endl;

        overall_avg_error += avg_error;
    }

    overall_avg_error /= numMeshes_POSE;
    double height_template_mesh = template_mesh->pCorner.y - template_mesh->nCorner.y;
    std::cout << "\tOverall Avg Error: " << overall_avg_error / height_template_mesh * 100 << "%" << std::endl;

    std::cout << std::endl;
}



/*
 * Test matrices Q
 *
 * Ground truth : i'th vertex posiiton in k'th mesh
 * Template position : i'th vertex posiiton in template mesh
 * Estimated position : R * S * Q * Template_position
 *
 * Error[i] = abs( Ground_truth - Estimated_position )
 * Avg Error = avg( Error[i] ), for all i in Vertices
 *
 * Percentage Error = Avg Error / Height of template mesh
 */
void testS()
{
    std::cout << "Testing S :" << std::endl;

    float overall_avg_error = 0.0;

    // size_t m = 1; {
    for( size_t m = 0; m < numMeshes_SHAPE; ++m ) {
        std::vector<glm::mat3> &matR = obj_SHAPE[m].matR;
        std::vector<double *> &deltaR = obj_SHAPE[m].deltaR;

        float max_error = 0.0;
        float avg_error = 0.0;

        for( int f = 0; f < numFaces; ++f ) {
            glm::vec3 &F = obj_SHAPE[0].Faces[f];
            glm::vec3 v1_real     = obj_SHAPE[m].Vertices[ F[1] ] - obj_SHAPE[m].Vertices[ F[0] ];
            glm::vec3 v2_real     = obj_SHAPE[m].Vertices[ F[2] ] - obj_SHAPE[m].Vertices[ F[0] ];
            glm::vec3 v1_         = obj_SHAPE[0].Vertices[ F[1] ] - obj_SHAPE[0].Vertices[ F[0] ];
            glm::vec3 v2_         = obj_SHAPE[0].Vertices[ F[2] ] - obj_SHAPE[0].Vertices[ F[0] ];
            glm::vec3 v1_computed = template_mesh->matR[faceBone[f]] * obj_SHAPE[m].matS[f]
                                    * template_mesh->matQ[f] * v1_;
            glm::vec3 v2_computed = template_mesh->matR[faceBone[f]] * obj_SHAPE[m].matS[f]
                                    * template_mesh->matQ[f] * v2_;

            glm::vec3 v1_diff = v1_computed - v1_real;
            glm::vec3 v2_diff = v2_computed - v2_real;

            max_error = std::max( max_error, glm::length( v1_diff ) );
            max_error = std::max( max_error, glm::length( v2_diff ) );
            avg_error += glm::length( v1_diff );
            avg_error += glm::length( v2_diff );
        }

        avg_error /= ( numFaces * 2 );
        // std::cout << "\t[ Mesh " << m << " ]\t\tMax Error: " << max_error << "\t\tAvg Error: "
        //           << avg_error << std::endl;

        overall_avg_error += avg_error;
    }

    overall_avg_error /= numMeshes_POSE;
    double height_template_mesh = template_mesh->pCorner.y - template_mesh->nCorner.y;
    std::cout << "\tOverall Avg Error: " << overall_avg_error / height_template_mesh * 100 << "%" << std::endl;

    std::cout << std::endl;
}



#endif //HUMAN_BODY_RECONSTRUCTION_SCAPE_OPERATIONS_HPP
