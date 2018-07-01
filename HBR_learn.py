import sys
import numpy as np
import cvxpy as cvx
from tqdm import tqdm, trange
from sklearn.decomposition import PCA


DATA_DIR = sys.argv[1]

READ_MATR_FROM_DISK = True
WRITE_MATR_TO_DISK  = False
READ_MATQ_FROM_DISK = True
WRITE_MATQ_TO_DISK  = False
READ_MATS_FROM_DISK = True
WRITE_MATS_TO_DISK  = False
READ_VECA_FROM_DISK = True
WRITE_VECA_TO_DISK  = False
READ_VECMU_VECSIGMA_MATU_FROM_DISK  = True
WRITE_VECMU_VECSIGMA_MATU_TO_DISK  = False

REGULARIZOR_LAMBDA = 1e-3

PRINT_ERROR_MATR = False
PRINT_ERROR_MATQ = False
PRINT_ERROR_MATS = False
PRINT_ERROR_VECA = False

# SOME HARD-CODING
if 'data1' in DATA_DIR :
    numMeshes_POSE = 71
    numMeshes_SHAPE = 37
    numVertices = 12500
    numFaces = 25000
    numBones = 16
    pathPLY_POSE = DATA_DIR + '/Mesh_POSE/'
    fileListPLY_POSE = [ str(i) + '.ply' for i in range(numMeshes_POSE) ]
    pathPLY_SHAPE = DATA_DIR + '/Mesh_SHAPE/'
    fileListPLY_SHAPE = [ str(i) + '.ply' for i in range(numMeshes_SHAPE) ]

elif 'data2' in DATA_DIR :
    numMeshes_POSE = 30
    numMeshes_SHAPE = 30
    numVertices = 20
    numFaces = 36
    numBones = 3
    pathPLY_POSE = DATA_DIR + '/Mesh_POSE/'
    fileListPLY_POSE = [
        'file_15_15.ply', 'file_15_30.ply', 'file_15_45.ply', 'file_15_60.ply', 'file_15_75.ply',
        'file_15_90.ply', 'file_30_15.ply', 'file_30_30.ply', 'file_30_45.ply', 'file_30_60.ply',
        'file_30_75.ply', 'file_30_90.ply', 'file_45_15.ply', 'file_45_30.ply', 'file_45_45.ply',
        'file_45_60.ply', 'file_45_75.ply', 'file_45_90.ply', 'file_60_15.ply', 'file_60_30.ply',
        'file_60_45.ply', 'file_60_60.ply', 'file_60_75.ply', 'file_60_90.ply', 'file_75_15.ply',
        'file_75_30.ply', 'file_75_45.ply', 'file_75_60.ply', 'file_75_75.ply', 'file_75_90.ply'
    ]
    pathPLY_SHAPE = DATA_DIR + '/Mesh_SHAPE/'
    fileListPLY_SHAPE = fileListPLY_POSE

elif 'data3' in DATA_DIR :
    numMeshes_POSE = 71
    numMeshes_SHAPE = 37
    numVertices = 6449
    numFaces = 12894
    numBones = 16
    pathPLY_POSE = DATA_DIR + '/Mesh_POSE/'
    fileListPLY_POSE = [ str(i) + '.ply' for i in range(numMeshes_POSE) ]
    pathPLY_SHAPE = DATA_DIR + '/Mesh_SHAPE/'
    fileListPLY_SHAPE = [ str(i) + '.ply' for i in range(numMeshes_SHAPE) ]


Faces = []
Vertices_POSE = []
Vertices_SHAPE = []
vertBone = []
faceBone = []
boneFaces = []
joinedBones = []
matR = [ [] for i in range(numMeshes_POSE) ]
matQ = [ 0 for i in range(numMeshes_POSE) ]
matS = [ 0 for i in range(numMeshes_SHAPE) ]
deltaR = []
vecA = [ 0 for i in range(numFaces) ]


# Read Mesh from PLY file
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


# Read skeleton from DATA_DIR / skeketonParts.dat
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


# Read joonedBones from DATA_DIR / joinedBoned.dat
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


def read_matQ( filename ) :
    mQ = []
    with open( filename, 'r' ) as file :
        for line in file :
            mQ.append( np.matrix(line).reshape((3, 3)).T )

    assert len(mQ) == numFaces, 'Error [read_matQ]: file {} has {} matrices whereas instead of ' \
        '{} matrices'.format( filename, len(mQ), numFaces )

    return np.array(mQ)


def read_matS( filename ) :
    mS = []
    with open( filename, 'r' ) as file :
        for line in file :
            mS.append( np.matrix(line).reshape((3, 3)).T )

    assert len(mS) == numFaces, 'Error [read_matS]: file {} has {} matrices instead of {} ' \
        'matrices!'.format( filename, len(mS), numFaces )

    return np.array(mS)


def read_deltaR( filename ) :
    dR = []
    with open( filename, 'r' ) as file :
        for line in file:
            dR.append( np.array(list(map( float, line.strip().split() ))) )
        return dR


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
        matU = np.zeros((numMeshes_SHAPE, 9 * numFaces))
        i = 0
        for line in file :
            matU[i] = np.fromstring( line, dtype=np.double, sep=' ' )
            i += 1

        return vecMu, vecSigma, matU

    return


def write_PLY(filename, V):
    with open(filename, 'w') as file:
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


def write_matQ( filename, mQ ) :
    with open( filename, 'w' ) as file :
        for f in range(numFaces) :
            file.write( ' '.join([ str(x) for x in mQ[f].T.flat ]) )
            file.write( '\n' )


def write_matS( filename, mS ) :
    with open( filename, 'w' ) as file :
        for f in range(numFaces) :
            file.write( ' '.join([ str(x) for x in mS[f].T.flat ]) )
            file.write( '\n' )


def write_vecA( filename, vA ) :
    with open( filename, 'w' ) as file :
        for f in range(numFaces) :
            file.write( ' '.join([ str(x) for x in vA[f].flatten() ]) )
            file.write( '\n' )


def write_vecMu_vecSigma_matU( filename, vecMu, vecSigma, matU ) :
    with open( filename, 'w' ) as file :
        # Writing vector Mu
        file.write( ' '.join([ str(x) for x in vecMu ]) )
        file.write( '\n\n' )
        # Writing vector Sigma
        file.write( ' '.join([ str(x) for x in vecSigma ]) )
        file.write( '\n\n' )
        # Writing PCA transformation matrix U
        for row in matU :
            file.write( ' '.join([ str(x) for x in row ]) )
            file.write( '\n' )


def test_matR( m, mR ) :
    if not PRINT_ERROR_MATR :
        return

    max_error = 0.0
    avg_error = 0.0

    for b in range(numBones) :
        for f in boneFaces[b] :
            F = Faces[f]
            v1_real = Vertices_POSE[m][ F[1] ] - Vertices_POSE[m][ F[0] ]
            v2_real = Vertices_POSE[m][ F[2] ] - Vertices_POSE[m][ F[0] ]
            v1_ = Vertices_POSE[0][ F[1] ] - Vertices_POSE[0][ F[0] ]
            v2_ = Vertices_POSE[0][ F[2] ] - Vertices_POSE[0][ F[0] ]
            v1_comp = np.matrix( mR[b] ) * v1_
            v2_comp = np.matrix( mR[b] ) * v2_

            error_v1 = np.linalg.norm( v1_comp - v1_real )
            error_v2 = np.linalg.norm( v2_comp - v2_real )
            max_error = max( max_error, error_v1, error_v2 )
            avg_error += ( error_v1 + error_v2 ) / ( 2 * numFaces )

    tqdm.write( '[ matR : Mesh {} ]\tAvg error: {:2.8f}'
                '\tMax error: {:2.8f}'.format( m, avg_error, max_error ) )

    return avg_error


def test_matQ( m, mQ ) :
    if not PRINT_ERROR_MATQ :
        return

    max_error = 0.0
    avg_error = 0.0

    for f in range(numFaces) :
        F = Faces[f]
        v1_real = Vertices_POSE[m][ F[1] ] - Vertices_POSE[m][ F[0] ]
        v2_real = Vertices_POSE[m][ F[2] ] - Vertices_POSE[m][ F[0] ]
        v1_ = Vertices_POSE[0][ F[1] ] - Vertices_POSE[0][ F[0] ]
        v2_ = Vertices_POSE[0][ F[2] ] - Vertices_POSE[0][ F[0] ]
        v1_comp = np.matrix( matR[m][faceBone[f]] ) * np.matrix( mQ[f] ) * v1_
        v2_comp = np.matrix( matR[m][faceBone[f]] ) * np.matrix( mQ[f] ) * v2_

        error_v1 = np.linalg.norm( v1_comp - v1_real )
        error_v2 = np.linalg.norm( v2_comp - v2_real )
        max_error = max( max_error, error_v1, error_v2 )
        avg_error += ( error_v1 + error_v2 ) / ( 2 * numFaces )

    tqdm.write( '[ matQ : Mesh {} ]\tAvg error: {:2.8}'
                '\tMax error: {:2.8}'.format( m, avg_error, max_error ) )

    return avg_error


def test_matS( m, mS ) :
    if not PRINT_ERROR_MATS :
        return

    max_error = 0.0
    avg_error = 0.0

    for f in range(numFaces) :
        F = Faces[f]
        v1_real = Vertices_SHAPE[m][ F[1] ] - Vertices_SHAPE[m][ F[0] ]
        v2_real = Vertices_SHAPE[m][ F[2] ] - Vertices_SHAPE[m][ F[0] ]
        v1_ = Vertices_SHAPE[0][ F[1] ] - Vertices_SHAPE[0][ F[0] ]
        v2_ = Vertices_SHAPE[0][ F[2] ] - Vertices_SHAPE[0][ F[0] ]
        v1_comp = ( np.matrix( matR[0][faceBone[f]] ) * np.matrix( mS[f] )
                    * np.matrix( matQ[0][f] ) * v1_ )
        v2_comp = ( np.matrix( matR[0][faceBone[f]] ) * np.matrix( mS[f] )
                    * np.matrix( matQ[0][f] ) * v2_ )

        error_v1 = np.linalg.norm( v1_comp - v1_real )
        error_v2 = np.linalg.norm( v2_comp - v2_real )
        max_error = max( max_error, error_v1, error_v2 )
        avg_error += ( error_v1 + error_v2 ) / ( 2 * numFaces )

    tqdm.write( '[ matS : Mesh {} ]\tAvg error: {:2.8f}'
                '\tMax error: {:2.8f}'.format( m, avg_error, max_error ) )

    return avg_error


def test_vecA() :
    if not PRINT_ERROR_VECA :
        return

    avg_error, max_error = 0.0, 0.0
    for f in range(numFaces) :
        aei = 0.0

        for m in range(numMeshes_POSE) :
            Q_real = matQ[m][f]
            Q_comp = np.array(
                [ np.dot( deltaR[m][faceBone[f]], vecA[f][i] ) for i in range(9) ]
            ).reshape(3, 3)

            error = np.linalg.norm( Q_comp - Q_real )
            max_error = max( max_error, error )
            aei += error / numMeshes_POSE

        # print( '\t[ Face {} Bone {} ]\tMax error: {}'
        #     '\tAvg error: {}'.format( f, faceBone[f], mei, aei ) )

        avg_error += aei / numFaces

    tqdm.write( '[ vecA ]\tAvg error: {:2.8f}'
                '\tMax error: {:2.8f}'.format( avg_error, max_error ) )

    return avg_error


# Make matrix M orthoginal and returns
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


def get_neighbor_faces() :
    assert len(Faces) > 0, 'Faces is empty!'
    assert len(boneFaces) > 0, 'boneFaces is empty!'

    # Computing all the faces belonging to a particular vertex, for all vertices
    vert_faces = [ [] for _ in range(numVertices) ]
    for f in range(len(Faces)) :
        for v in Faces[f] :
            vert_faces[v].append( f )

    # Computing the neighboring faces of a particular face, for all faces
    neighbor_faces = [ set() for _ in range(numFaces) ]
    for f in range(numFaces) :
        for v in Faces[f] :
            neighbor_faces[f].update([ f2 for f2 in vert_faces[v] if faceBone[f] == faceBone[f2] ])
        neighbor_faces[f].remove( f )

    return neighbor_faces


if __name__ == '__main__' :
    #
    # Read Meshes and Skeleton
    #
    for filename in tqdm( fileListPLY_POSE, desc='Reading Pose-Space Meshes', ncols=100,
                          leave=False ) :
        V, F = read_PLY( pathPLY_POSE + filename )
        Faces = F
        Vertices_POSE.append( V )
    Vertices_POSE = np.array( Vertices_POSE )
    print( 'Reading Pose-Space Meshes ... done!' )

    for filename in tqdm( fileListPLY_SHAPE, desc='Reading Shape-Space Meshes', ncols=100,
                          leave=False ) :
        V, F = read_PLY( pathPLY_SHAPE + filename )
        Faces = F
        Vertices_SHAPE.append( V )
    Vertices_SHAPE = np.array( Vertices_SHAPE )
    print( 'Reading Shape-Space Meshes ... done!' )

    vertBone, boneFaces, faceBone = read_Skeleton( DATA_DIR + '/skeletonParts.dat' )
    joinedBones = read_joinedBones( DATA_DIR + '/joinedBones.dat' )
    neighbor_faces = get_neighbor_faces()
    print( 'Reading skeleton data ... done!' )
    print()

    #
    # Solving and Testing matrices R
    #
    pathMatR = DATA_DIR + '/matR/'

    if READ_MATR_FROM_DISK :

        for m in trange( numMeshes_POSE, desc='Reading matR', ncols=100, leave=False ) :
            filename = str(m) + '.dat'
            matR[m] = read_matR( pathMatR + filename )
            test_matR( m, matR[m] )
        matR = np.array( matR )
        print( 'Reading matR ... done!' )

    else :

        for m in trange( numMeshes_POSE, desc='Computing matR', ncols=100, leave=False ) :
            for b in range(numBones) :
                R = cvx.Variable(3, 3)
                err = 0
                for f in boneFaces[b] :
                    F = Faces[f]
                    v1_ = Vertices_POSE[0][ F[1] ] - Vertices_POSE[0][ F[0] ]
                    v2_ = Vertices_POSE[0][ F[2] ] - Vertices_POSE[0][ F[0] ]
                    v1 = Vertices_POSE[m][ F[1] ] - Vertices_POSE[m][ F[0] ]
                    v2 = Vertices_POSE[m][ F[2] ] - Vertices_POSE[m][ F[0] ]
                    err += cvx.sum_squares( ( R * v1_ ) - v1 )
                    err += cvx.sum_squares( ( R * v2_ ) - v2 )
                obj = cvx.Minimize( err )
                prob = cvx.Problem( obj )
                prob.solve()
                matR[m].append( orthogonal( R.value ) )

            matR[m] = np.array(matR[m])
            test_matR( m, matR[m] )

        matR = np.array( matR )
        print( 'Computing matR ... done!' )

        if WRITE_MATR_TO_DISK :
            for m in trange( numMeshes_POSE, desc='Writing matR to disk', ncols=100,
                             leave=False ) :
                filename = pathMatR + str(m) + '.dat'
                write_matR( filename, matR[m] )
            print( 'Writing matR to disk ... done!' )

        print()

    #
    # Computing deltaR for meshes
    #
    for m in trange( numMeshes_POSE, desc='Computing deltaR', ncols=100, leave=False ) :
        dR = []
        for b in range(numBones) :
            b1, b2 = joinedBones[b]

            # Method 2, according to C++ code
            # check file scape_operations.hpp for more details
            R1 = np.dot( matR[m][b1], matR[m][b].T )
            R2 = np.dot( matR[m][b2], matR[m][b].T )
            trace1 = R1[0, 0] + R1[1, 1] + R1[2, 2]
            trace2 = R2[0, 0] + R2[1, 1] + R2[2, 2]
            theta1 = np.arccos( np.clip( (trace1 - 1.0) / 2, -0.99999, 0.99999 ) )
            theta2 = np.arccos( np.clip( (trace2 - 1.0) / 2, -0.99999, 0.99999 ) )

            t1 = np.array([ R1[2, 1] - R1[1, 2], R1[0, 2] - R1[2, 0], R1[1, 0] - R1[0, 1] ]) \
                * abs(theta1) / ( 2 * np.sin(theta1) )
            t2 = np.array([ R2[2, 1] - R2[1, 2], R2[0, 2] - R2[2, 0], R2[1, 0] - R2[0, 1] ]) \
                * abs(theta2) / ( 2 * np.sin(theta2) )

            dR.append([ t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], 1 ])

        deltaR.append( dR )

    deltaR = np.array( deltaR )
    print( 'Computing deltaR ... done!' )

    #
    # Solving and Testing matrices Q
    #
    pathMatQ = DATA_DIR + '/matQ/'

    if READ_MATQ_FROM_DISK :

        for m in trange( numMeshes_POSE, desc='Reading matQ', ncols=100, leave=False ) :
            filename = str(m) + '.dat'
            mQ = read_matQ( pathMatQ + filename )
            matQ[m] = mQ
            test_matQ( m, matQ[m] )
        matR = np.array( matQ )
        print( 'Reading matQ ... done!' )

    else :

        def double_check( f1, f2 ) :
            if (f1, f2) in double_check_set or (f2, f1) in double_check_set :
                return False
            double_check_set.add( (f1, f2) )
            return True

        for m in trange( numMeshes_POSE, desc='Computing matQ', ncols=100, leave=False ) :
            Q = [ cvx.Variable(3, 3) for i in range(numFaces) ]

            double_check_set = set()

            for b in range(numBones) :
                err1 = 0
                err2 = 0

                # Solving for actual values of Q
                for f in range(len(boneFaces[b])) :
                    fi = boneFaces[b][f]
                    F = Faces[fi]
                    v1_ = Vertices_POSE[0][ F[1] ] - Vertices_POSE[0][ F[0] ]
                    v2_ = Vertices_POSE[0][ F[2] ] - Vertices_POSE[0][ F[0] ]
                    v1 = Vertices_POSE[m][ F[1] ] - Vertices_POSE[m][ F[0] ]
                    v2 = Vertices_POSE[m][ F[2] ] - Vertices_POSE[m][ F[0] ]

                    err1 += cvx.sum_squares( ( np.matrix( matR[m][b] ) * Q[fi] * v1_ ) - v1 )
                    err1 += cvx.sum_squares( ( np.matrix( matR[m][b] ) * Q[fi] * v2_ ) - v2 )

                # Adding regularizer for Q
                if REGULARIZOR_LAMBDA > 1e-9 :
                    for f1 in boneFaces[b] :
                        for f2 in neighbor_faces[f1] :
                            if double_check( f1, f2 ) :
                                err2 += cvx.sum_squares( Q[f1] - Q[f2] )

                obj = cvx.Minimize( err1 + REGULARIZOR_LAMBDA * err2 )
                prob = cvx.Problem( obj )
                prob.solve()

            matQ[m] = np.array([ q.value for q in Q ])
            test_matQ( m, matQ[m] )

        matQ = np.array( matQ )
        print( 'Computing matQ ... done!' )

        if WRITE_MATQ_TO_DISK :
            for m in trange( numMeshes_POSE, desc='Writing matQ to disk', ncols=100,
                             leave=False ) :
                filename = pathMatQ + str(m) + '.dat'
                write_matQ( filename, matQ[m] )
            print( 'Writing matQ to disk ... done!' )

        print()

    #
    # Solving and Testing matrices S
    #
    pathMatS = DATA_DIR + '/matS/'

    if READ_MATS_FROM_DISK :

        for m in trange( numMeshes_SHAPE, desc='Reading matS', ncols=100, leave=False ) :
            filename = str(m) + '.dat'
            mS = read_matS( pathMatS + filename )
            matS[m] = mS
            test_matS( m, matS[m] )
            matS = np.array( matS )
        print( 'Reading matS ... done!' )

    else :

        for m in trange( numMeshes_SHAPE, desc='Computing matS', ncols=100, leave=False ) :
            S = [ cvx.Variable( 3, 3 ) for i in range(numFaces) ]
            err1 = 0
            err2 = 0

            double_check_set = set()

            # Solving for actual values of S
            for f in range(numFaces) :
                F = Faces[f]
                b = faceBone[f]
                v1_ = Vertices_SHAPE[0][ F[1] ] - Vertices_SHAPE[0][ F[0] ]
                v2_ = Vertices_SHAPE[0][ F[2] ] - Vertices_SHAPE[0][ F[0] ]
                v1  = Vertices_SHAPE[m][ F[1] ] - Vertices_SHAPE[m][ F[0] ]
                v2  = Vertices_SHAPE[m][ F[2] ] - Vertices_SHAPE[m][ F[0] ]

                err1 += cvx.sum_squares(
                    ( np.matrix(matR[0][b]) * S[f] * np.matrix(matQ[0][f]) * v1_ ) - v1
                )
                err1 += cvx.sum_squares(
                    ( np.matrix(matR[0][b]) * S[f] * np.matrix(matQ[0][f]) * v2_ ) - v2
                )

            # Adding regularizer for Q
            if REGULARIZOR_LAMBDA > 1e-9 :
                for f1 in range(numFaces) :
                    for f2 in neighbor_faces[f1] :
                        if double_check( f1, f2 ) :
                            err2 += cvx.sum_squares( S[f1] - S[f2] )

            obj = cvx.Minimize( err1 + REGULARIZOR_LAMBDA * err2 )
            prob = cvx.Problem( obj )
            prob.solve()

            matS[m] = np.array([ s.value for s in S ])

            test_matS( m, matS[m] )

        matS = np.array( matS )

        print( 'Computing matS ... done!' )

        if WRITE_MATS_TO_DISK :
            for m in trange( numMeshes_SHAPE, desc='Writing matS to disk', ncols=100,
                             leave=False ) :
                filename = pathMatS + str(m) + '.dat'
                # print( 'Writing file', filename )
                write_matS( filename, matS[m] )
            print( 'Writing matS to disk ... done!' )

        print()

    #
    # Solving and Testing vectors A
    #
    if READ_VECA_FROM_DISK :

        vecA = read_vecA( DATA_DIR + '/vecA.dat' )
        test_vecA()
        print( 'Reading vecA ... done!' )

    else :

        for f in trange( numFaces, desc='Computing vecA', ncols=100, leave=False) :
            vA = []

            for i in range(9) :

                A = cvx.Variable(7)
                err = 0
                for m in range(numMeshes_POSE) :
                    err += cvx.square(
                        ( deltaR[m][faceBone[f]] * A ) - np.matrix( matQ[m][f] ).A1[i]
                    )
                obj = cvx.Minimize( err )
                prob = cvx.Problem( obj )
                prob.solve()

                vA.append( A.value.A1 )

            vecA[f] = np.array( vA )

        vecA = np.array( vecA )
        test_vecA()

        print( 'Computing vecA ... done!' )

        if WRITE_VECA_TO_DISK :
            write_vecA( DATA_DIR + '/vecA.dat', vecA )
            print( 'Writing vecA to disk ... done!' )

        print()

    #
    # Solving and Testing for PCA parameters U, Mu and Sigma
    #
    if READ_VECMU_VECSIGMA_MATU_FROM_DISK:

        vecMu, vecSigma, matU = read_vecMu_vecSigma_matU( DATA_DIR + '/pca_parameters.dat' )
        print( 'Reading PCA parameters ... done!' )

    else :

        matS_vectorised = np.matrix([ matS[i].flatten() for i in range(len(matS)) ])
        matS_pca = PCA().fit( matS_vectorised )

        vecMu = matS_pca.mean_
        vecSigma = matS_pca.explained_variance_
        matU = matS_pca.components_

        print( 'Computing PCA parameters ... done!' )

        if WRITE_VECMU_VECSIGMA_MATU_TO_DISK :
            write_vecMu_vecSigma_matU( DATA_DIR + '/vecMu_vecSigma_matU.dat',
                                       vecMu, vecSigma, matU )
            print( 'Writing PCA parameters to disk ... done!' )
