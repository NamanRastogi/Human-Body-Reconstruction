from numpy import argmax
import sys
import xml.etree.ElementTree as ET

########################################################################

collada_file = sys.argv[1]
tree = ET.parse(collada_file)
root = tree.getroot()

num_bones_per_vert = root[3][0][0][5][2].text
bones = root[3][0][0][5][3].text
weights = root[3][0][0][3][0].text

num_bones_per_vert = list(map( int, num_bones_per_vert.strip().split() ))
bones = list(map( int, bones.strip().split() ))[::2]
weights = list(map( float, weights.strip().split() ))

num_vertices = len(num_bones_per_vert)
num_bones = len(set(bones))

print( 'num_bones_per_vert :', num_vertices, sum(num_bones_per_vert) )
print( 'bones :', len(bones), num_bones )
print( 'weights :', len(weights) )
print()

########################################################################

bones_per_vert = []
weights_per_vert = []
counter = 0
for num_bones_per_vert_i in num_bones_per_vert :
    x = bones[ counter : counter + num_bones_per_vert_i ]
    y = weights[ counter : counter + num_bones_per_vert_i ]
    bones_per_vert.append( x )
    weights_per_vert.append( y )
    counter += num_bones_per_vert_i

print( 'bones_per_vert :', len(bones_per_vert) )
print( 'weights_per_vert :', len(weights_per_vert) )
print()

########################################################################

bone_of_verts = []
for vi in range(num_vertices) :
    max_weight_idx = argmax( weights_per_vert[vi] )
    bone_of_verts.append( bones_per_vert[vi][max_weight_idx] )

verts_of_bones = [[]] * num_bones
for vi in range(num_vertices) :
    verts_of_bones[ bone_of_verts[vi] ].append( vi )

print('bone_of_verts :', len(bone_of_verts))
print( 'verts_of_bones :', len(verts_of_bones) )
print()

########################################################################

filename = 'skeletonParts.dat'
print( 'Writing file', filename )

with open( filename, 'w' ) as file :
    file.write( '{} {}\n'.format( num_vertices, num_bones ) )
    file.write( ' '.join( str(x) for x in bone_of_verts) )

print('Writing file', filename, '...', 'done!' )
