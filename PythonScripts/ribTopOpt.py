from abaqus import *
from abaqusConstants import *
import __main__
import section
import odbSection
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import displayGroupOdbToolset as dgo
from math import atan, sin, cos, tan, radians
import numpy as np
import glob # for retrieving the last odb file in the SAVE.odb directory
import os   # for retrieving the last odb file in the SAVE.odb directory
from ast import literal_eval as make_tuple # for using make_tuple

def generateAirfoilCoordinates(normalizedCoordinates, chord, locationAlongWingspan, leadingEdgeSweepAngle): 

    # Read in the airfoil coordinates from the .CSV file
    with open(normalizedCoordinates) as file:
        stringCoordinates = [line.rstrip('\n') for line in file]

    # Convert the coordinates from strings to tuples and remove all white space.
    tupledCoordinates = [ make_tuple(''.join(coordinate.split())) for coordinate in stringCoordinates]

    # Initialize the scaled coordinates matrix
    numberRows = len(tupledCoordinates)
    numberColumns = len(tupledCoordinates[0])
    scaledCoordinates = [[0 for column in xrange(numberColumns)] for row in xrange(numberRows)] 
    
    # Scale the coordinates based on the chord
    for index, coordinateList in enumerate(tupledCoordinates):
        scaledCoordinates[index] = [chord*coordinate for coordinate in coordinateList]

    # Iterate over each x-coordinate and add the x offset
    xOffset = locationAlongWingspan*tan(leadingEdgeSweepAngle)
    for index, row in enumerate(scaledCoordinates):
        scaledCoordinates[index][0] += xOffset

    return scaledCoordinates


# Create a new database
Mdb()

########## (1) Sketch geometry and create parts ##########

partName = 'Rib'
rootFolder = 'C:\\temp\\hunterNewDirectory\\'
airfoilCoordinates = rootFolder + 'CoordinateData\\normalizedCoordinates.csv'
coordinates = generateAirfoilCoordinates(airfoilCoordinates, 1, 0, 32)

# We only need the x and y coordinates for the sketch, so delete the z coordinate
for coordinate3D in coordinates:
    del coordinate3D[-1]

# Extract the top and bottom coordinates because spline will mess up the trailing edge if sketched all at once.
topCoordinates = coordinates[0:103]
bottomCoordinates = coordinates[103:]

# Start the sketch
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__',  sheetSize=1000)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)

# Sketch the top and bottom separately
s.Spline(points = topCoordinates)
s.Spline(points = bottomCoordinates)

# Sketch a line to connect the trailing edge
s.Line(point1 = topCoordinates[-1], point2 = bottomCoordinates[0])

# Create the part
p = mdb.models['Model-1'].Part(name=partName, dimensionality=THREE_D, 
    type=DEFORMABLE_BODY)
p = mdb.models['Model-1'].parts[partName]
p.BaseShell(sketch = s)
s.unsetPrimaryObject()
p = mdb.models['Model-1'].parts[partName]
del mdb.models['Model-1'].sketches['__profile__']

########## (2) Create material and assign section ##########

# Create rib material
materialName = 'Aluminum2024'
materialDensity = 0.1
materialModulus = 10600000
materialPoissonsRatio = 0.3
mdb.models['Model-1'].Material(name=materialName)
mdb.models['Model-1'].materials[materialName].Density(table=((materialDensity, ), ))
mdb.models['Model-1'].materials[materialName].Elastic(table=((materialModulus, 
    materialPoissonsRatio), ))

# Create the section
sectionName = 'ribSection'
ribThickness = 0.1
mdb.models['Model-1'].HomogeneousShellSection(name=sectionName, 
preIntegrate=OFF, material=materialName, thicknessType=UNIFORM, 
thickness=ribThickness, thicknessField='', nodalThicknessField='', 
idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
integrationRule=SIMPSON, numIntPts=5)

# Assign the section
p = mdb.models['Model-1'].parts[partName]
f = p.faces
faces = f.findAt(((0.5, 0.0, 0.0), ))
region = p.Set(faces=faces, name='ribSet')
p = mdb.models['Model-1'].parts[partName]
p.SectionAssignment(region=region, sectionName='ribSection', offset=0.0, 
    offsetType=MIDDLE_SURFACE, offsetField='', 
    thicknessAssignment=FROM_SECTION)

########## (3) Partition the face at the spar locations ##########

# Create datum planes
p = mdb.models['Model-1'].parts['Rib']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.155)
p = mdb.models['Model-1'].parts['Rib']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.45)
p = mdb.models['Model-1'].parts['Rib']
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.695)

# Partition the face
p = mdb.models['Model-1'].parts['Rib']
f = p.faces
pickedFaces = f.findAt(((0.001012, 0.0, 0.0), ))
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[3], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Rib']
f = p.faces
pickedFaces = f.findAt(((0.22547, -0.010066, 0.0), ))
d2 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d2[4], faces=pickedFaces)
p = mdb.models['Model-1'].parts['Rib']
f = p.faces
pickedFaces = f.findAt(((0.579144, -0.003393, 0.0), ))
d1 = p.datums
p.PartitionFaceByDatumPlane(datumPlane=d1[5], faces=pickedFaces)

########## (4) Assemble parts ##########

# Assemble part
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts[partName]
a.Instance(name='Rib-1', part=p, dependent=ON)

# Create mesh
p = mdb.models['Model-1'].parts['Rib']
p.seedPart(size=0.005, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['Rib']
f = p.faces
pickedRegions = f.findAt(((0.613333, 0.012003, 0.0), ), ((0.400824, 0.010174, 
    0.0), ), ((0.128341, -0.006748, 0.0), ), ((0.745801, 0.000228, 0.0), ))
p.setMeshControls(regions=pickedRegions, technique=STRUCTURED)
elemType1 = mesh.ElemType(elemCode=S4R, elemLibrary=STANDARD, 
    secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
elemType2 = mesh.ElemType(elemCode=S3, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['Rib']
f = p.faces
faces = f.findAt(((0.613333, 0.012003, 0.0), ), ((0.400824, 0.010174, 0.0), ), 
    ((0.128341, -0.006748, 0.0), ), ((0.745801, 0.000228, 0.0), ))
pickedRegions =(faces, )
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
p = mdb.models['Model-1'].parts['Rib']
p.generateMesh()

########## (5) Define steps ##########

mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

########## (6) Create loads and boundary conditions ##########

# Create the top front load
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Rib-1'].edges
side1Edges1 = s1.findAt(((0.035185, 0.015327, 0.0), ))
region = a.Surface(side1Edges=side1Edges1, name='Surf-3')
mdb.models['Model-1'].ShellEdgeLoad(name='frontTopLoad', 
    createStepName='Step-1', region=region, magnitude=-1.0, 
    distributionType=UNIFORM, field='', localCsys=None)

# Create the rest of the load on the top
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Rib-1'].edges
side1Edges1 = s1.findAt(((0.511278, 0.028774, 0.0), ), ((0.228719, 0.02816, 
    0.0), ), ((0.771341, 0.017305, 0.0), ))
region = a.Surface(side1Edges=side1Edges1, name='Surf-4')
mdb.models['Model-1'].ShellEdgeLoad(name='topMiddleLoad', 
    createStepName='Step-1', region=region, magnitude=-0.5, 
    distributionType=UNIFORM, field='', localCsys=None)

# Create the front bottom load
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Rib-1'].edges
side1Edges1 = s1.findAt(((0.114962, -0.023354, 0.0), ))
region = a.Surface(side1Edges=side1Edges1, name='Surf-5')
mdb.models['Model-1'].ShellEdgeLoad(name='frontBottomLoad', 
    createStepName='Step-1', region=region, magnitude=0.75, 
    distributionType=UNIFORM, field='', localCsys=None)

# Create the rest of the load on the bottom
a = mdb.models['Model-1'].rootAssembly
s1 = a.instances['Rib-1'].edges
side1Edges1 = s1.findAt(((0.633803, -0.019878, 0.0), ), ((0.376241, -0.030044, 
    0.0), ), ((0.923717, -0.002852, 0.0), ))
region = a.Surface(side1Edges=side1Edges1, name='Surf-6')
mdb.models['Model-1'].ShellEdgeLoad(name='Load-4', createStepName='Step-1', 
    region=region, magnitude=0.4, distributionType=UNIFORM, field='', 
    localCsys=None)

# Create boundary conditions
a = mdb.models['Model-1'].rootAssembly
e1 = a.instances['Rib-1'].edges
edges1 = e1.findAt(((0.695, 0.012556, 0.0), ), ((0.45, 0.015075, 0.0), ), ((
    0.155, 0.012755, 0.0), ))
region = a.Set(edges=edges1, name='Set-2')
mdb.models['Model-1'].EncastreBC(name='BC-1', createStepName='Step-1', 
    region=region, localCsys=None)

########## (7) Create and run job ##########

ModelName='Model-1'

mdb.Job(name=ModelName, model=ModelName, description='', type=ANALYSIS, 
    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    scratch='', multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)

job = mdb.jobs[ModelName]

# delete lock file, which for some reason tends to hang around, if it exists
if os.access('%s.lck'%ModelName,os.F_OK):
    os.remove('%s.lck'%ModelName)
    
# Run the job, then process the results.        
job.submit()
job.waitForCompletion()
print 'Completed job'

########## (8) Create and run optimization job ##########

a = mdb.models['Model-1'].rootAssembly
f1 = a.instances['Rib-1'].faces
mdb.models['Model-1'].TopologyTask(name='TopOpt', region=MODEL, 
    materialInterpolationTechnique=SIMP)
mdb.models['Model-1'].optimizationTasks['TopOpt'].setValues(freezeLoadRegions = ON, freezeBoundaryConditionRegions=ON)

### Design Responses
mdb.models['Model-1'].optimizationTasks['TopOpt'].SingleTermDesignResponse(
    name='DRESP_Strain_Energy', region=MODEL, identifier='STRAIN_ENERGY', 
    drivingRegion=None, operation=SUM, stepOptions=())
mdb.models['Model-1'].optimizationTasks['TopOpt'].SingleTermDesignResponse(
    name='DRESP_Volume', region=MODEL, identifier='VOLUME', drivingRegion=None, 
    operation=SUM, stepOptions=())

### Objective function
mdb.models['Model-1'].optimizationTasks['TopOpt'].ObjectiveFunction(
    name='obj_max_stiffness', objectives=((OFF, 'DRESP_Strain_Energy', 1.0, 
    0.0, ''), ))

### Volume constraint
mdb.models['Model-1'].optimizationTasks['TopOpt'].OptimizationConstraint(
    name='constraint_volume', designResponse='DRESP_Volume', 
    restrictionMethod=RELATIVE_LESS_THAN_EQUAL, restrictionValue=0.7)

### Create and submit the optimization process job
optimizationName = 'OptProcess' 
prototypeJobName = optimizationName + '-Job'
optimizedResultLocation = rootFolder + optimizationName

mdb.OptimizationProcess(name=optimizationName, model=ModelName, 
    task='TopOpt', description='', prototypeJob=prototypeJobName, 
    maxDesignCycle=25, odbMergeFrequency=2, 
    dataSaveFrequency=OPT_DATASAVE_EVERY_CYCLE)

mdb.optimizationProcesses[optimizationName].Job(
    name=prototypeJobName, model=ModelName, atTime=None, waitMinutes=0, 
    waitHours=0, queue=None, memory=90, memoryUnits=PERCENTAGE, 
    getMemoryFromAnalysis=True, multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)

# Get a picture of the result
'''
session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
    CONTOURS_ON_DEF, ))
session.viewports['Viewport: 1'].odbDisplay.commonOptions.setValues(
    visibleEdges=FEATURE)
session.viewports['Viewport: 1'].odbDisplay.commonOptions.setValues(
    uniformScaleFactor=1)
session.printOptions.setValues(vpBackground=ON)
picturePath = rootFolder + 'topOptPictures\\picture'
session.printToFile(fileName=picturePath, format=PNG, canvasObjects=(session.viewports['Viewport: 1'], ))
'''

# delete lock file, which for some reason tends to hang around, if it exists
lockFile = 'C:/temp/Hunter/' + optimizationName + '/SAVE.odb/*.lck' # * means all if need specific format then *.csv
if os.access(lockFile, os.F_OK):
    os.remove(lockFile)

optimizationJob = mdb.optimizationProcesses[optimizationName]
optimizationJob.submit()
optimizationJob.waitForCompletion()
print 'Completed optimization job'