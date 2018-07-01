import sys
import numpy as np
import cvxpy as cvx

if len(sys.argv) < 5 :
    print( 'Usage:', sys.argv[0], '<DATA_DIR>', '<POINT_CLOUD_FILE>', '<ACTUAL_MESH_NAME>',
           '<OUTPUT_MESH_NAME>' )
    exit()

DATA_DIR = sys.argv[1]
POINT_CLOUD_FILE = sys.argv[2]
ACTUAL_MESH_NAME = sys.argv[3]
OUTPUT_MESH_NAME = sys.argv[4]

LAMBDA = 0.5


if 'data1' in DATA_DIR :
    numMeshes_POSE = 71
    numMeshes_SHAPE = 37
    numVertices = 12500
    numFaces = 25000
    numBones = 16

elif 'data2' in DATA_DIR :
    numMeshes_POSE = 30
    numMeshes_SHAPE = 30
    numVertices = 20
    numFaces = 36
    numBones = 3

elif 'data3' in DATA_DIR :
    numMeshes_POSE = 71
    numMeshes_SHAPE = 37
    numVertices = 6449
    numFaces = 12894
    numBones = 16


Vertices_template = []
Faces = []
vertBone = []
faceBone = []
boneFaces = []
joinedBones = []
vecA = []
vecMu, vecSigma, matU = [], [], []

Vertices_estimate = []
ptcld = []


def read_PLY( filename ) :
    with open( filename, 'r' ) as file :
        line = next(file).strip()
        while( line != 'end_header' ) :
            line = next(file).strip()
            if line.split()[0] == 'element' :
                if line.split()[1] == 'vertex' :
                    nV = int( line.split()[2] )
                elif line.split()[1] == 'face' :
                    nF = int( line.split()[2] )
        V = []
        F = []
        for _ in range(nV) :
            V.append( np.array([
                list(map( float, next(file).strip().split() )) ]).T
            )
        for _ in range(nF) :
            F.append( list(map( int, next(file).strip().split() ))[1:] )

        assert nV == numVertices, 'Error [read_PLY]: file {} has {} vertices instead of {} ' \
            'vertices!'.format( filename, nV, numVertices )
        assert nF == numFaces, 'Error [read_PLY]: file {} has {} faces instead of {} ' \
            'faces!'.format( filename, nF, numFaces )

        return np.array(V), np.array(F)


def read_Skeleton( filename ) :
    # Reading skeletonParts.dat
    with open( filename, 'r' ) as file :
        nV, nB = list(map( int, file.readline().strip().split() ))
        vB = list(map( int, file.readline().strip().split() ))

        assert nV == numVertices and len(vB) == numVertices, 'Error [read_Skeleton]: file {} ' \
            'has {} vertices instead of {} vertices!'.format( filename, len(vB), numVertices )
        assert nB == numBones and ( max(vB) + 1 )  == numBones, 'Error [read_Skeleton]: file {} ' \
            'has {} bones instead of {} bones!'.format( filename, len(vB), numVertices )

    # Generating faceBones
    fB = []
    for f in Faces :
        fB.append( vB[ f[0] ] )

    # Generating boneFaces
    bF = [ [] for _ in range(numBones) ]
    for i, b in enumerate( fB ) :
        bF[ b ].append( i )

    return np.array(vB), np.array(bF), np.array(fB)


def read_joinedBones( filename ) :
    jB = []
    with open( filename, 'r' ) as file :
        nB = int( file.readline().strip() )

        assert nB == numBones, 'Error [read_joinedBones]: file has {} bones instead of {} ' \
            'bones!'.format( nB, numBones )

        for line in file :
            jB.append( list(map( int, line.strip().split() )) )

        assert len(jB) == numBones, 'Error [read_joinedBones]: file has {} bones ' \
            'instead of {} bones!'.format( len(jB), numBones )

        return np.array( jB )


def read_matR( filename ) :
    mR = []
    with open( filename, 'r' ) as file :
        for line in file :
            mR.append( np.matrix(line).reshape((3, 3)).T )

    assert len(mR) == numBones, 'Error [read_matR]: file {} has {} matrices instead of {} ' \
        'matrices!'.format( filename, len(mR), numBones )

    return np.array( mR )


def read_vecA( filename ) :
    with open( filename, 'r' ) as file :
        vA = []
        for _ in range(numFaces) :
            vA.append( np.array( next(file).strip().split(), dtype=np.float64 ).reshape(9, 7) )
        return np.array(vA)


def read_vecMu_vecSigma_matU( filename ) :
    with open( filename, 'r' ) as file :
        # Reading vector Mu
        line = file.readline().strip()
        vecMu = np.fromstring( line, dtype=np.double, sep=' ' )
        file.readline()  # removing blank line
        # Reading vector Sigma
        line = file.readline()
        vecSigma = np.fromstring( line, dtype=np.double, sep=' ' )
        file.readline()  # removing blank line
        # Reading PCA transformation matrix U
        matU = []
        for line in file :
            matU.append( np.fromstring( line, dtype=np.double, sep=' ' ) )
        matU = np.array( matU )

        return matU, vecMu, vecSigma

    return


def load_point_cloud( filename ) :
    ptcld = []
    with open( filename, 'r' ) as file :
        file.readline()  # file header, to be discarded
        for line in file :
            v = np.array(list(map( float, line.strip().split()[:3] )))
            i = int( line.strip().split()[3] )
            ptcld.append(( v, i ))
    return np.array(ptcld)


def write_PLY( filename, V ):
    with open( filename, 'w' ) as file:
        file.write(
            'ply\n'
            'format ascii 1.0\n'
            'element vertex {}\n'
            'property float x\n'
            'property float y\n'
            'property float z\n'
            'element face {}\n'
            'property list uchar int vertex_indices\n'
            'end_header\n'.format(numVertices, numFaces)
        )
        for v in V:
            file.write('{} {} {}\n'.format(v[0][0], v[1][0], v[2][0]))
        for f in Faces:
            file.write('3 {} {} {}\n'.format(f[0], f[1], f[2]))
        file.write('\n')


def write_matR( filename, mR2 ) :
    with open( filename, 'w' ) as file :
        for f in range(numBones) :
            file.write( ' '.join([ str(x) for x in mR2[f].T.flat ]) )
            file.write( '\n' )


def orthogonal( M ) :
    assert M.shape == (3, 3), \
        'Error [orthogonal]: input matrix has shape {} instead of (3,3)'.format( M.shape )
    M = np.matrix(M)
    MtM = M.T @ M
    w, v = np.linalg.eigh( MtM )

    temp = np.zeros_like( MtM )  # temp is for storing MtM^(-1/2)
    for i in range(3) :
        temp += v[:, i] @ v[:, i].T / np.sqrt( w[i] )

    M_orthogonal = M @ temp
    return np.array( M_orthogonal )


def initialize_matR( ptcld ) :
    bone_points = [ [] for _ in range(numBones) ]
    for i, ( v, correspondance ) in enumerate(ptcld) :
        bone_points[ vertBone[correspondance] ].append( i )

    mR = []
    for b in range(numBones) :
        R = cvx.Variable(3, 3)
        T = cvx.Variable(3)
        err = 0
        for i in range(len(bone_points[b])) :
            gt_v = ptcld[ bone_points[b][i] ][0].reshape(3, 1)
            template_v = Vertices_template[ ptcld[ bone_points[b][i] ][1] ]
            err += cvx.sum_squares( ( R * template_v ) + T - gt_v )
        obj = cvx.Minimize( err )
        prob = cvx.Problem( obj )
        prob.solve()
        mR.append( orthogonal( R.value ) )

    return np.array( mR )


def generate_deltaR( matR ) :
    dR = []
    for b in range(numBones) :
        b1, b2 = joinedBones[b]

        # Method 2, according to C++ code
        # check file scape_operations.hpp for more details
        R1 = matR[b1] @ matR[b].T
        R2 = matR[b2] @ matR[b].T
        trace1 = R1[0, 0] + R1[1, 1] + R1[2, 2]
        trace2 = R2[0, 0] + R2[1, 1] + R2[2, 2]
        theta1 = np.arccos( np.clip( (trace1 - 1.0) / 2, -0.99999, 0.99999 ) )
        theta2 = np.arccos( np.clip( (trace2 - 1.0) / 2, -0.99999, 0.99999 ) )

        t1 = np.array([ R1[2, 1] - R1[1, 2], R1[0, 2] - R1[2, 0], R1[1, 0] - R1[0, 1] ]) \
            * abs(theta1) / ( 2 * np.sin(theta1) )
        t2 = np.array([ R2[2, 1] - R2[1, 2], R2[0, 2] - R2[2, 0], R2[1, 0] - R2[0, 1] ]) \
            * abs(theta2) / ( 2 * np.sin(theta2) )

        dR.append([ t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], 1 ])

    return np.array( dR )


def update_matR( v_mesh ) :
    mR = []
    for b in range(numBones) :
        R = cvx.Variable(3, 3)
        err = 0
        for f in boneFaces[b] :
            F = Faces[f]
            v1_ = Vertices_template[ F[1] ] - Vertices_template[ F[0] ]
            v2_ = Vertices_template[ F[2] ] - Vertices_template[ F[0] ]
            v1 = v_mesh[ F[1] ] - v_mesh[ F[0] ]
            v2 = v_mesh[ F[2] ] - v_mesh[ F[0] ]
            err += cvx.sum_squares( ( R * v1_ ) - v1 )
            err += cvx.sum_squares( ( R * v2_ ) - v2 )
        obj = cvx.Minimize( err )
        prob = cvx.Problem( obj )
        prob.solve()
        mR.append( orthogonal( R.value ) )

    return np.array( mR )


def estimate_mesh_on_ptcld( matR, vecB, ptcld ) :
    Y = [ cvx.Variable(3) for _ in range(numVertices) ]
    err1 = 0.0
    err2 = 0.0

    deltaR = generate_deltaR( matR )
    matQ = np.array([
        np.array(
            [ np.dot( deltaR[faceBone[f]], vecA[f][i] ) for i in range(9) ]
        ).reshape(3, 3) for f in range(numFaces)
    ])

    matS = ( np.dot( matU.T, vecB ) + vecMu ).reshape( numFaces, 3, 3 )

    for f in range(numFaces) :
        F = Faces[f]

        R_ = matR[ faceBone[f] ]
        Q_ = matQ[ f ]
        S_ = matS[ f ]
        RSQ = np.dot( np.dot( R_, S_ ), Q_ )

        v1 = ( Vertices_template[ F[1] ] - Vertices_template[ F[0] ] )
        v2 = ( Vertices_template[ F[2] ] - Vertices_template[ F[0] ] )
        yk1 = Y[ F[1] ] - Y[ F[0] ]
        yk2 = Y[ F[2] ] - Y[ F[0] ]

        err1 += cvx.sum_squares( np.dot(RSQ, v1) - yk1 )
        err1 += cvx.sum_squares( np.dot(RSQ, v2) - yk2 )

    for gt_v, i in ptcld :
        err2 += cvx.norm( gt_v - Y[i] )

    obj = cvx.Minimize( err1 + LAMBDA * err2 )
    prob = cvx.Problem( obj )
    prob.solve()

    return np.array([ Y[i].value for i in range(numVertices) ]).reshape( numVertices, 3, 1 )


def update_vecB( matR, v_mesh, matU, vecMu ) :

    vB = cvx.Variable(numMeshes_SHAPE)
    err = 0

    deltaR = generate_deltaR( matR )
    matQ = np.array([
        np.array(
            [ np.dot( deltaR[faceBone[f]], vecA[f][i] ) for i in range(9) ]
        ).reshape(3, 3) for f in range(numFaces)
    ])

    for f in range(numFaces) :
        F = Faces[f]

        R_ = matR[ faceBone[f] ]
        Q_ = matQ[ f ]
        S_ = cvx.reshape(
            matU.T[ f * 9 : ( f + 1 ) * 9 ] * vB + vecMu[ f * 9 : ( f + 1 ) * 9 ],
            3, 3
        )

        RSQ = ( R_ * S_ ) * Q_
        v1 = ( Vertices_template[ F[1] ] - Vertices_template[ F[0] ] )
        v2 = ( Vertices_template[ F[2] ] - Vertices_template[ F[0] ] )
        yk1 = v_mesh[ F[1] ] - v_mesh[ F[0] ]
        yk2 = v_mesh[ F[2] ] - v_mesh[ F[0] ]

        err += cvx.sum_squares( RSQ * v1 - yk1 )
        err += cvx.sum_squares( RSQ * v2 - yk2 )

    obj = cvx.Minimize( err )
    prob = cvx.Problem( obj )
    prob.solve()

    return vB.value.A1


def generate_final_mesh( matR, vecB ) :
    Y = [ cvx.Variable(3) for _ in range(numVertices) ]
    err = 0

    deltaR = generate_deltaR( matR )
    matQ = np.array([
        np.array(
            [ np.dot( deltaR[faceBone[f]], vecA[f][i] ) for i in range(9) ]
        ).reshape(3, 3) for f in range(numFaces)
    ])

    matS = ( np.dot( matU.T, vecB ) + vecMu ).reshape( numFaces, 3, 3 )

    for f in range(numFaces) :
        F = Faces[f]

        R_ = matR[ faceBone[f] ]
        Q_ = matQ[ f ]
        S_ = matS[ f ]
        RSQ = np.dot( np.dot( R_, S_ ), Q_ )

        v1 = ( Vertices_template[ F[1] ] - Vertices_template[ F[0] ] )
        v2 = ( Vertices_template[ F[2] ] - Vertices_template[ F[0] ] )
        yk1 = Y[ F[1] ] - Y[ F[0] ]
        yk2 = Y[ F[2] ] - Y[ F[0] ]

        err += cvx.sum_squares( np.dot(RSQ, v1) - yk1 )
        err += cvx.sum_squares( np.dot(RSQ, v2) - yk2 )

    obj = cvx.Minimize( err )
    prob = cvx.Problem( obj )
    prob.solve()

    return np.array([ Y[i].value for i in range(numVertices) ]).reshape( numVertices, 3, 1 )


def estimation_error_ptcld( ptcld, v_mesh ) :
    error = 0.0
    for gt_v, i in ptcld :
        error += np.linalg.norm( gt_v - v_mesh[i].reshape(3) )

    error /= len(ptcld)
    return error / ( np.max( Vertices_template[:, 1] ) - np.min( Vertices_template[:, 1] ) ) * 100


def estimation_error_mesh( v_mesh_1, v_mesh_2 ) :
    error = 0.0
    for a, b in zip( v_mesh_1, v_mesh_2 ) :
        error += np.linalg.norm( a - b )
    error /= len( v_mesh_1 )

    return error / ( np.max( Vertices_template[:, 1] ) - np.min( Vertices_template[:, 1] ) ) * 100


def main() :
    global Vertices_template, Faces, vertBone, faceBone, boneFaces, joinedBones
    global vecA, vecMu, vecSigma, matU

    #
    # Read Template mesh and Skeleton
    #
    template_fileName = DATA_DIR + "/Mesh_POSE/0.ply"
    Vertices_template, Faces = read_PLY( template_fileName )
    print( 'Reading Template Mesh ... done!' )

    vertBone, boneFaces, faceBone = read_Skeleton( DATA_DIR + '/skeletonParts.dat' )
    joinedBones = read_joinedBones( DATA_DIR + '/joinedBones.dat' )
    print( 'Reading skeleton data ... done!' )

    #
    # Loading vecA
    #
    vecA = read_vecA( DATA_DIR + '/vecA.dat' )
    print( 'Reading vecA ... done!' )

    #
    # Loading PCA parameters
    #
    # matU.shape = (37, 116046)
    # vecMu.shape = (116046,)
    # vecSigma.shape = (37,)
    matU, vecMu, vecSigma = read_vecMu_vecSigma_matU( DATA_DIR + '/pca_parameters.dat' )
    print( 'Reading PCA parameters ... done!' )

    #
    # Load Point Cloud (.ptcld)
    #
    ptcld = load_point_cloud( POINT_CLOUD_FILE )

    #
    # Initial Error
    #
    print( 'Initial error:', estimation_error_ptcld( ptcld, Vertices_template ), '%' )
    print()

    #
    # Initialize matR and vecB
    #
    matR = initialize_matR( ptcld )
    # write_matR( input('Filename for matR: '), matR )
    vecB = np.zeros( shape=(numMeshes_SHAPE), dtype=np.float32 )
    print( 'Initializing matR and vecB ... done!' )

    #
    # Optimisation loop
    #
    for loop_count in range(5) :

        # Update matR
        # if loop_count > 0 :
        #     print( 'Updating matR' )
        #     matR = update_matR( Vertices_estimate )
        #     print( 'Updating matR ... done!' )
        #     print()

        Vertices_estimate = generate_final_mesh( matR, vecB )
        print( 'Estimation error:', estimation_error_ptcld( ptcld, Vertices_estimate ), '%' )
        print()

        # Estimating Mesh on Point Cloud
        print( 'Estimating mesh on Point Cloud' )
        Vertices_estimate = estimate_mesh_on_ptcld( matR, vecB, ptcld )
        print( 'Estimating mesh on Point Cloud ... done!' )

        print( 'Estimation error:', estimation_error_ptcld( ptcld, Vertices_estimate ), '%' )
        print()

        # Updating vecB
        print( 'Updating vecB' )
        vecB = update_vecB( matR, Vertices_estimate, matU, vecMu )
        print( 'Updating vecB ... done!' )
        print( 'vecB:', vecB / vecSigma )

    #
    # Generating Final Mesh
    #
    Vertices_estimate = generate_final_mesh( matR, vecB )
    print( 'Estimation error:', estimation_error_ptcld( ptcld, Vertices_estimate ), '%' )

    #
    # Error of generated mesh with actual mesh
    # actual mesh: mesh from which point cloud is made
    #
    Vertices_actual, _ = read_PLY( ACTUAL_MESH_NAME )
    print( 'Final Estimation error:',
           estimation_error_mesh( Vertices_actual, Vertices_estimate ), '%' )

    #
    # Writing final mesh onto disk
    #
    print( 'Writing Mesh', OUTPUT_MESH_NAME )
    write_PLY( OUTPUT_MESH_NAME, Vertices_estimate )
    print( 'Writing Mesh', OUTPUT_MESH_NAME, '...done!' )
    print()


if __name__ == '__main__' :
    main()
