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
#from Post_P_Script import getResults
import numpy as np
import glob # for retrieving the last odb file in the SAVE.odb directory
import os   # for retrieving the last odb file in the SAVE.odb directory
from ast import literal_eval as make_tuple # for using make_tuple

def calculateLoadCondition(loadCase):
    if loadCase == 'takeoff':
        # Define the plane characteristics for this load case
        planeWeight = 138700.0               # lbf
        wingArea = feetToInches(1570*12)     # square in
        maxLoad = planeWeight/(2.0*wingArea) # psi

    if loadCase == 'supersonicCruise':
        # Define the plane characteristics for this load case
        planeWeight = 131000.0                 # lbf
        wingArea = feetToInches(1212*12)       # square in
        maxLoad = planeWeight/(2.0*wingArea) # psi

    return planeWeight, wingArea, maxLoad

def generateAirfoilCoordinates(chord, locationAlongWingspan, normalizedCoordinates, leadingEdgeSweepAngle): 

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

def chord(z, wingDictionary):

    # Extract the wing data
    wingArea = wingDictionary['wingArea']
    taperRatio = wingDictionary['taperRatio']
    wingSpan = wingDictionary['wingSpan']

    # Equation from wikipedia page: Chord (aeronautics). Calculate the chord at a distance along the wingspan z
    chord = ((2*wingArea)/((1 + taperRatio)*wingSpan))*(1 - (1 - taperRatio)*abs(2*z)/wingSpan)

    return chord

def feetToInches(x):
    return x*12.0

def computeSweepAngleAt(desiredChordLocation, knownChordLocation, knownSweepAngle, wingDictionary):

    aspectRatio = wingDictionary['aspectRatio']
    taperRatio = wingDictionary['taperRatio']

    # Equation from Vadali's notes on wing geometry
    desiredSweepAngle = atan( tan(knownSweepAngle) - (4.0/aspectRatio)*(desiredChordLocation - knownChordLocation)*(1.0 - taperRatio)/(1.0 + taperRatio) )

    return float(desiredSweepAngle*(180.0/pi))

def getPointCoordinatesOnObject(obj):
    pointOnObjectTuple = obj.pointOn[0]
    pointOnObjectList = list(pointOnObjectTuple)

    return pointOnObjectList

def generateCompositeDictionary(materialName, density, numberOfLayers, skinThickness, laminaTable):
    return {'material': materialName, 'density': density, 'layerThickness': skinThickness/float(numberOfLayers), 'laminaTable': laminaTable }

def generateMaterialDictionary(materialName, youngsModulus, density, poissonsRatio):
    return {'material': materialName, 'modulusOfElasticity': youngsModulus, 'density': density, 'poissonsRatio': poissonsRatio}

def generateMeshDictionary(instanceName, seedSize, order, technique):
    return { 'instanceName': instanceName, 'seedSize': seedSize, 'order': order, 'technique': technique}

def computeStringerLocations(stringersPerRegion, sparLocations):

    stringerLocations = [None] * (len(sparLocations) - 1)
    for index in xrange(0, len(sparLocations) - 1):

        # Get the location for the current spar and the next spar
        currentSparLocation = sparLocations[index]
        nextSparLocation = sparLocations[index + 1]

        # Evenly distribute the stringers between both of the spars
        stringerLocations[index] = np.linspace(currentSparLocation + 0.02, nextSparLocation, stringersPerRegion, endpoint=False).tolist()

    # stringerLocations is a list of lists, so flatten it out
    stringerLocations = [stringerLocation for sublist in stringerLocations for stringerLocation in sublist]
    
    return stringerLocations

def computeSweptXValue(pointAlongWingspan, percentChord, wingDictionary):

    # Extract the data for the wing
    wingSpan = wingDictionary['wingSpan']
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']
    rootChord = wingDictionary['rootChord']
    tipChord = wingDictionary['tipChord']

    sweptXValue = (2.0/wingSpan)*((wingSpan/2.0)*tan(leadingEdgeSweep) + (percentChord)*(tipChord - rootChord))*pointAlongWingspan + (percentChord*rootChord)

    return sweptXValue

def generateRibs(ribDictionary, sparDictionary, wingDictionary):

    print('Creating rib parts')

    # Extract the variables from the dictionaries
    numberOfRibs = ribDictionary['numberOfRibs']
    material = ribDictionary['material']
    thickness = ribDictionary['thickness']
    circledDiameterPercentage = ribDictionary['circleDiameter']
    airfoilCoordinates = ribDictionary['airfoilCoordinates']
    sparLocations = sparDictionary['sparLocations']
    wingSpan = wingDictionary['wingSpan']
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']

    # Calculate the distance between each rib, the location of each rib, and the chord at each location
    ribStepSize = (wingSpan/2)/(numberOfRibs - 1)
    ribLocations = [ ribStepSize*(i) for i in xrange(0, numberOfRibs) ]
    ribChords = [ chord(ribLocation, wingDictionary) for ribLocation in ribLocations ]

    # Instance array to store the names of each rib
    ribNames = ['' for iter in xrange(numberOfRibs)]

    # Create a rib part for the desired number of ribs
    for i in xrange(numberOfRibs):

        # Name the rib 
        ribNames[i] = 'rib' + str(i+1)
            
        # Get the coordinates for this rib
        ribChord = ribChords[i]
        ribLocation = ribLocations[i]
        coordinates = generateAirfoilCoordinates(ribChord, ribLocation, airfoilCoordinates, leadingEdgeSweep)
        
        # We only need the x and y coordinates for the sketch, so delete the z coordinate
        for coordinate3D in coordinates:
            del coordinate3D[-1]
        
        # Extract the top and bottom coordinates because spline will mess up the trailing edge if sketched all at once.
        topCoordinates = coordinates[0:103]
        bottomCoordinates = coordinates[103:]
        camberLineCoordinates = [ [coordinate[0], 0] for coordinate in topCoordinates ]
        xValues = [ coordinate[0] for coordinate in coordinates ]
        yValues = [ coordinate[1] for coordinate in coordinates ]
        
        # Calculate the the camber line coordinates so we can place the circles on the camber line
        for index in xrange(0, len(topCoordinates)):

            # To compare the same coordinates, we must go forwards through topCoordinates, and backwards through bottomCoordinates
           camberLineCoordinates[index][1] = (0.5)*(topCoordinates[index][1] + bottomCoordinates[-(index+1)][1])

        # Start the sketch
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__',  sheetSize=1000)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.setPrimaryObject(option=STANDALONE)
        
        # Sketch the top and bottom separately
        s.Spline(points = topCoordinates)
        s.Spline(points = bottomCoordinates)
        # s.Spline(points = camberLineCoordinates) # Sketch the camber line
        
        # Sketch a line to connect the trailing edge
        s.Line(point1 = topCoordinates[-1], point2 = bottomCoordinates[0])
        # s.Line(point1 = topCoordinates[-1], point2 = camberLineCoordinates[-1])   # This is if we plot the camber line
        # s.Line(point1 = camberLineCoordinates[-1], point2 = bottomCoordinates[0]) # This is if we plot the camber line
        
        # Calculate the rib height and radius of the circle
        ribHeight = 0.06*ribChord # the rib is a NACA 0406, so it has a max height of 6% of its chord
        circleDiameterInches = circledDiameterPercentage*ribHeight
        circleRadiusInches = circleDiameterInches/2.0
        
        # Calculate the x location of the circle's center, and find the closest x value in the set of coordinates
        hole1ChordLocation = (0.5)*(sparLocations[1] - sparLocations[0]) + sparLocations[0]
        hole2ChordLocation = (0.5)*(sparLocations[2] - sparLocations[1]) + sparLocations[1]
    
        hole1LocationInches = hole1ChordLocation*ribChord + topCoordinates[0][0]
        hole2LocationInches = hole2ChordLocation*ribChord + topCoordinates[0][0]

        hole1Coordinates = min(camberLineCoordinates, key=lambda coordinate:abs(coordinate[0] - hole1LocationInches))
        hole2Coordinates = min(camberLineCoordinates, key=lambda coordinate:abs(coordinate[0] - hole2LocationInches))
        
        '''
        # TODO: Make sure the holes meet the geometric bounds
        # Iterate over holes 1 and 2 and make sure they are within the geometric bounds of the rib
        holeLocations = [hole1Coordinates[0], hole2Coordinates[0]]
        ribTopYValues = [0, 0]
        ribBottomYValues = [0, 0]
        for index, holeLocation in enumerate(holeLocations):
            ribTopYValues[index] = topCoordinates[holeLocation]
            ribBottomYValues[index] = bottomCoordinates[holeLocation]

        # if the circle overlaps the airfoil, get rid of the circle. Effectively this will increase the mass of the wing and thus make us not consider this design.
        if hole >=  
        '''

        # Create the circles
        s.CircleByCenterPerimeter(center=hole1Coordinates, point1=(hole1Coordinates[0], circleRadiusInches))
        s.CircleByCenterPerimeter(center=hole2Coordinates, point1=(hole2Coordinates[0], circleRadiusInches))

        # Autotrim the camber line inside the circles. THIS DOESN'T WORK, THIS IS AN ABAQUS ISSUE, NOT A CODE ISSUE.
        # s.autoTrimCurve(curve1=g.findAt(hole1Coordinates), point1=hole1Coordinates)
        # s.autoTrimCurve(curve1=g.findAt(hole2Coordinates), point1=hole2Coordinates)
        
        # Create the part
        p = mdb.models['Model-1'].Part(name=ribNames[i], dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p = mdb.models['Model-1'].parts[ribNames[i]]
        p.BaseShell(sketch = s)
        s.unsetPrimaryObject()
        p = mdb.models['Model-1'].parts[ribNames[i]]
        del mdb.models['Model-1'].sketches['__profile__']
        
        # Before creating the partitions, create a set for the root and tip ribs so we can easily loft the skin later
        if i == 0:
            p = mdb.models['Model-1'].parts[ribNames[i]]
            f, e = p.faces, p.edges
            outerEdges = e[2:]                                # e has the form: [hole2, hole1, bottomEdge, trailingEdge, topEdge]
            p.Set(edges=outerEdges, name='rootRibOuterEdges') # Create a set for the outer edges
            #p.Set(faces=f, name='rootRibFaces')           # Create a set for the faces

        if i == numberOfRibs - 1:
            p = mdb.models['Model-1'].parts[ribNames[i]]
            f, e = p.faces, p.edges
            outerEdges = e[2:]                               # e has the form: [hole2, hole1, bottomEdge, trailingEdge, topEdge]
            p.Set(edges=outerEdges, name='tipRibOuterEdges') # Create a set for the outer edges
            #p.Set(faces=f, name='tipRibFaces')           # Create a set for the faces

        # Create a horizontal datum plane passing through the xz plane
        p = mdb.models['Model-1'].parts[ribNames[i]]
        p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
        mdb.models['Model-1'].parts[ribNames[i]].features.changeKey(fromName='Datum plane-1', 
            toName='horizontalDatumPlane')

        # Create a vertical datum plane passing through the origin of the first hole
        p = mdb.models['Model-1'].parts[ribNames[i]]
        p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=hole1Coordinates[0])
        mdb.models['Model-1'].parts[ribNames[i]].features.changeKey(fromName='Datum plane-1', 
            toName='hole1DatumPlane')

        # Create a vertical datum plane passing through the origin of the first hole
        p = mdb.models['Model-1'].parts[ribNames[i]]
        p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=hole2Coordinates[0])
        mdb.models['Model-1'].parts[ribNames[i]].features.changeKey(fromName='Datum plane-1', 
            toName='hole2DatumPlane')

        # TODO: Partition the ribs, for some reason this causes the stringers to break. 
        # For each datum plane, partition the rib at the plane
        datumPlaneNames = ['hole1DatumPlane', 'hole2DatumPlane', 'horizontalDatumPlane']
        for datumName in datumPlaneNames:
            p = mdb.models['Model-1'].parts[ribNames[i]]
            f = p.faces
            datumID = p.features[datumName].id
            datumPlane = p.datums[datumID]
            p.PartitionFaceByDatumPlane(datumPlane=datumPlane, faces=f)
        
    # Create rib material
    materialName = material['material']
    materialDensity = material['density']
    materialModulus = material['modulusOfElasticity']
    materialPoissonsRatio = material['poissonsRatio']
    
    mdb.models['Model-1'].Material(name=materialName)
    mdb.models['Model-1'].materials[materialName].Density(table=((materialDensity, ), ))
    mdb.models['Model-1'].materials[materialName].Elastic(table=((materialModulus, 
        materialPoissonsRatio), ))

    # Create the rib section
    sectionName = 'ribSection'
    mdb.models['Model-1'].HomogeneousShellSection(name=sectionName, 
    preIntegrate=OFF, material=materialName, thicknessType=UNIFORM, 
    thickness=thickness, thicknessField='', nodalThicknessField='', 
    idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)

    # Assign the rib section to each rib
    for rib in ribNames:
        nameOfCoordinateSet = rib + 'Set'

        # Get the rib's face
        p = mdb.models['Model-1'].parts[rib]
        faces = p.faces

        # Assign the section to the rib
        region = p.Set(faces=faces, name=nameOfCoordinateSet)
        p = mdb.models['Model-1'].parts[rib]
        p.SectionAssignment(region=region, sectionName=sectionName, offset=0.0, 
            offsetType=MIDDLE_SURFACE, offsetField='', 
            thicknessAssignment=FROM_SECTION)

    # For each rib name in the list, instance the rib
    for index, ribName in enumerate(ribNames):

        instanceName = ribName + '-1'

        # Instance the rib
        a = mdb.models['Model-1'].rootAssembly
        a.DatumCsysByDefault(CARTESIAN)
        p = mdb.models['Model-1'].parts[ribName]
        a.Instance(name=instanceName, part=p, dependent=OFF)

        # Get the z location of the rib leading edge
        zOffset = ribLocations[index]

        # Translate the rib to its place along the wingspan
        a = mdb.models['Model-1'].rootAssembly
        a.translate(instanceList=(instanceName, ), vector=(0.0, 0.0, zOffset))

    # Turn all the ribs into one part
    a = mdb.models['Model-1'].rootAssembly 
    allRibInstances = list(a.instances.values()) # Get all of the values from the a.instances dictionary (these are the rib instance objects)
    a.InstanceFromBooleanMerge(name='AllRibs', instances=allRibInstances, originalInstances=SUPPRESS, domain=GEOMETRY)

    # Unsupress the root and tip rib so the skin can be created
    tipRibInstanceName = 'rib' + str(numberOfRibs) + '-1'
    a = mdb.models['Model-1'].rootAssembly
    a.features['rib1-1'].resume()
    a.features[tipRibInstanceName].resume()

def generateSkin(ribDictionary, skinDictionary, rootChord):
    
    print('Creating the skin')

    # Extract data from dictionaries
    numberOfRibs = ribDictionary['numberOfRibs']
    material = skinDictionary['material']
    thickness = skinDictionary['thickness']

    # Get the first and last rib instances
    a = mdb.models['Model-1'].rootAssembly

    rootRibName = 'rib1-1'
    tipRibName = 'rib' + str(numberOfRibs) + '-1'

    rootRib = a.instances['rib1-1']
    tipRib = a.instances[tipRibName]
    
    # From the assembly module, create a part with the root rib and tip rib
    a.InstanceFromBooleanMerge(name='Skin', instances=(rootRib, tipRib, ), originalInstances=SUPPRESS, domain=GEOMETRY)

    # Unsuppress the instances so we can use them when we 
    a = mdb.models['Model-1'].rootAssembly
    a.features[rootRibName].resume()
    a.features[tipRibName].resume()

    # Loft the edges on the tip rib and root rib
    p = mdb.models['Model-1'].parts['Skin']
    f1, e1 = p.faces, p.edges
    rootEdges = tuple(p.sets['rootRibOuterEdges'].edges)
    tipEdges = tuple(p.sets['tipRibOuterEdges'].edges)
    p.ShellLoft(loftsections=( rootEdges, tipEdges ), startCondition=NONE, endCondition=NONE )

    # Rename the loft feature
    mdb.models['Model-1'].parts['Skin'].features.changeKey(fromName='Shell Loft-1', 
    toName='Skin')

    # Go to the Skin part
    p = mdb.models['Model-1'].parts['Skin']
    f = p.faces

    # Specify the set names for the root and tip rib
    rootRibSetName = 'rib1Set'
    tipRibSetName = 'rib' + str(numberOfRibs) + 'Set'

    # Get the face of each rib
    rootRibFaces = p.sets[rootRibSetName].faces
    tipRibFaces = p.sets[tipRibSetName].faces

    # Create a list of the faces
    faceList = rootRibFaces + tipRibFaces

    # Delete the root and tip rib faces
    p.RemoveFaces(faceList=faceList, deleteCells=False)

    # Create skin material
    sectionName = 'skinSection'

    ##### TODO: Automate the skin material process #####
    '''
    materialName = material['material']
    materialDensity = material['density']
    mdb.models['Model-1'].Material(name=materialName)
    mdb.models['Model-1'].materials[materialName].Density(table=((materialDensity, ), ))
    '''
    '''
    if materialName == 'Aluminum 2024':
        print('Aluminum')
        materialDensity = material['density']
        materialModulus = material['modulusOfElasticity']
        materialPoissonsRatio = material['poissonsRatio']

        # Assign the material properties for Aluminum 2024
        mdb.models['Model-1'].materials[materialName].Elastic(table=((materialModulus, 
            materialPoissonsRatio), ))

        # Create the skin section
        sectionName = 'skinSection'
        mdb.models['Model-1'].HomogeneousShellSection(name=sectionName, preIntegrate=OFF, material=materialName, thicknessType=UNIFORM, thickness=thickness, thicknessField='', nodalThicknessField='', idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, numIntPts=5)
    '''
    #if materialName == 'Composite':
    #print('composite')
    '''
    layerThickness = material['layerThickness']
    a1 = material['laminaTable'][0]
    a2 = material['laminaTable'][1]
    a3 = material['laminaTable'][2]
    a4 = material['laminaTable'][3]
    a5 = material['laminaTable'][4]
    a6 = material['laminaTable'][5]
    
    # Assign the material properties for the composite section
    mdb.models['Model-1'].materials[materialName].Elastic(type = LAMINA, table=((a1, a2, a3, a4, a5, a6), ))

    sectionLayer1 = section.SectionLayer(material='composite', thickness=layerThickness, orientAngle=0.0, numIntPts=3, plyName='')
    sectionLayer2 = section.SectionLayer(material='composite', thickness=layerThickness, orientAngle=90.0, numIntPts=3, plyName='')
    sectionLayer3 = section.SectionLayer(material='composite', thickness=layerThickness, orientAngle=0.0, numIntPts=3, plyName='')
    sectionLayer4 = section.SectionLayer(material='composite', thickness=layerThickness, orientAngle=90.0, numIntPts=3, plyName='')
    sectionLayer5 = section.SectionLayer(material='composite', thickness=layerThickness, orientAngle=0.0, numIntPts=3, plyName='')

    # Create the skin section
    mdb.models['Model-1'].CompositeShellSection(name=materialName, preIntegrate=OFF, idealization=NO_IDEALIZATION, symmetric=False, thicknessType=UNIFORM, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, layup=(sectionLayer1, sectionLayer2, sectionLayer3, sectionLayer4, sectionLayer5, ))
    '''

    ##define material
    #mdb.models['Model-1'].Material(name='composite')
    #mdb.models['Model-1'].materials['composite'].Density(table=((0.05708, ), ))
    #mdb.models['Model-1'].materials['composite'].Elastic(type=LAMINA, table=((
    #    20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0), ))

    ##The ALUMINUM 2024 section needs to be commented out and replace it with this 	
    ##make composite section (CHANGE THICKNESS FRMO HARDCODE TO READ THE BOUNDS	OF WING THICKNESS)
    ##define material
    #mdb.models['Model-1'].Material(name='composite')
    #mdb.models['Model-1'].materials['composite'].Density(table=((0.05708, ), ))
    #mdb.models['Model-1'].materials['composite'].Elastic(type=LAMINA, table=((
    #    20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0), ))

    ##The ALUMINUM 2024 section needs to be commented out and replace it with this 	
    ##make composite section (CHANGE THICKNESS FRMO HARDCODE TO READ THE BOUNDS	OF WING THICKNESS)
     ##define material
    mdb.models['Model-1'].Material(name='composite')
    mdb.models['Model-1'].materials['composite'].Density(table=((0.05708, ), ))
    mdb.models['Model-1'].materials['composite'].Elastic(type=LAMINA, table=((
        20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0), ))

    ##The ALUMINUM 2024 section needs to be commented out and replace it with this 	
    ##make composite section (CHANGE THICKNESS FRMO HARDCODE TO READ THE BOUNDS	OF WING THICKNESS)
    sectionLayer1 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=0.0, numIntPts=3, plyName='')
    sectionLayer2 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=90.0, numIntPts=3, plyName='')
    sectionLayer3 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=45.0, numIntPts=3, plyName='')
    sectionLayer4 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=-45.0, numIntPts=3, plyName='')
    sectionLayer5 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=-45.0, numIntPts=3, plyName='')
    sectionLayer6 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=45.0, numIntPts=3, plyName='')
    sectionLayer7 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=90.0, numIntPts=3, plyName='')
    sectionLayer8 = section.SectionLayer(material='composite', thickness=thickness/8.0, 
        orientAngle=0.0, numIntPts=3, plyName='')
    mdb.models['Model-1'].CompositeShellSection(name=sectionName, 
        preIntegrate=OFF, idealization=NO_IDEALIZATION, symmetric=False, 
        thicknessType=UNIFORM, poissonDefinition=DEFAULT, thicknessModulus=None, 
        temperature=GRADIENT, useDensity=OFF, integrationRule=SIMPSON, layup=(
        sectionLayer1, sectionLayer2, sectionLayer3, sectionLayer4, sectionLayer5, sectionLayer6, sectionLayer7, sectionLayer8, 
        ))

    # Get the skin's faces
    p = mdb.models['Model-1'].parts['Skin']
    faces = p.faces

    # Assign the section to the skin
    region = p.Set(faces=faces, name='skinSet')
    p = mdb.models['Model-1'].parts['Skin']
    p.SectionAssignment(region=region, sectionName=sectionName, offset=0.0, 
        offsetType=TOP_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    '''
    # Get the top skin faces furthest aft
    p = mdb.models['Model-1'].parts['Skin']
    allFaces = p.faces
    topFaces = [ face for face in allFaces if ((face.pointOn[0][2] > 0) and (face.pointOn[0][0] > 0.6*rootChord)) ]
    bucklingFace = [topFaces[2]]
    p.Set(faces=part.FaceArray(bucklingFace), name='bucklingFace')
    '''

    # Instance the skin
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts['Skin']
    a.Instance(name='Skin-1', part=p, dependent=ON)

    # Supress the root and tip rib instances since we only needed them to generate the skin
    tipRibInstanceName = 'rib' + str(numberOfRibs) + '-1'
    a = mdb.models['Model-1'].rootAssembly
    a.features['rib1-1'].suppress()
    a.features[tipRibInstanceName].suppress()

def partitionRibs(numberOfRibs):
    ribPartNames = [ 'rib' + str(i) for i in xrange(1, numberOfRibs) ]

    for ribName in ribPartNames:

        # Iterate over each datum plane and partition the rib using that datum plane
        p = mdb.models['Model-1'].parts[ribName]
        facesToBePartitioned = p.faces
        datumPlaneNames = ['horizontalDatumPlane','hole1DatumPlane', 'hole2DatumPlane']

        for index, datumPlaneName in enumerate(datumPlaneNames):

            # Select the datum plane for the current spar
            datumID = p.features[datumPlaneName].id
            datumPlane = p.datums[datumID]
            
            # Partition each face on the part for the current datum plane
            p.PartitionFaceByDatumPlane(datumPlane=datumPlane, faces=facesToBePartitioned)

            # Name the partition face
            #partitionName = 'partitionFace' + str(index+1)
            #partitionFaceNames[index] = partitionName
            #mdb.models['Model-1'].parts[ribNames[i]].features.changeKey(fromName='Partition face-1', toName=partitionName)

'''
def createTheSparPartitions(sparLocations):

    print('Partitioning the ribs where the spars are going to be')

    # Change the default datum csys-1 to be called the reference frame. This will help with indexing datum objects later
    mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum csys-1', toName='referenceFrame')
    
    # Define d1 to be the coordinate system datum object
    a = mdb.models['Model-1'].rootAssembly
    desiredID = a.features['referenceFrame'].id
    d1 = a.datums[desiredID]

    # Create a helper point above the origin
    a = mdb.models['Model-1'].rootAssembly
    v1 = a.instances['rib1-1'].vertices
    a.DatumPointByOffset(point=v1.findAt(coordinates=(0.0, 0.0, 0.0)), vector=(0.0, 1.0, 0.0))
    mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum pt-1', toName='HelperPointAboveOrigin')

    # Initialize an array to store the names of the spar datum planes
    sparDatumPlanes = [None] * len(sparLocations)

    # For each spar location, create a datum plane that cuts through the ribs at the spar location
    for index, sparLocation in enumerate(sparLocations):

        # Name the current datum axis and the datum plane
        axisName = 'spar' + str(index + 1) + 'DatumAxis'
        helperDatumPlaneName = 'spar' + str(index + 1) + 'HelperPlane'
        sparDatumPlaneName = 'spar' + str(index + 1) + 'DatumPlane'
        sparDatumPlanes[index] = sparDatumPlaneName

        # Create a helper datum axis representing the spar's direction from the root rib to the tip rib
        sparSweepAngle = computeSweepAngleAt(sparLocation, 0, leadingEdgeSweep)
        a.DatumAxisByRotation(line=d1.axis3, axis=d1.axis2, angle=sparSweepAngle)
        mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum axis-1', 
            toName=axisName)

        # Create a helper plane using the datum axis and the helper point above the origin
        a = mdb.models['Model-1'].rootAssembly
        lineID = a.features[axisName].id
        pointID = a.features['HelperPointAboveOrigin'].id
        line = a.datums[lineID]
        point = a.datums[pointID]
        a.DatumPlaneByLinePoint(line=line, point=point)
        mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum plane-1', 
            toName=helperDatumPlaneName)

        # Offset a datum plane so it aligns where the current spar should be at along the ribs
        ####### TODO: THE LOCATIONS OF THE DATUM PLANES ARE OFF FOR SOME REASON. sparLocationInches is correct, but when abaqus offsets the datum planes they're not in the right spot.
        # The (-7.4) is a temporary fix
        sparLocationInches = sparLocation*rootChord - 7.4
        print(sparLocationInches)
        a = mdb.models['Model-1'].rootAssembly
        helperPlaneID = a.features[helperDatumPlaneName].id
        helperPlane = a.datums[helperPlaneID]
        a.DatumPlaneByOffset(plane=helperPlane, flip=SIDE2, offset=sparLocationInches)
        mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum plane-1', 
            toName=sparDatumPlaneName)

    # Get the dictionary of instances and initialize the ribFaces list
    a = mdb.models['Model-1'].rootAssembly
    instancesDictionary = a.instances
    
    # Iterate over each datum plane representing a spar and create a partition on the skin's face
    for index1, datumPlaneName in enumerate(sparDatumPlanes):

        # Collect all the faces to be partitioned for this spar
        ribFaces = [None] * len(instancesDictionary)

        # Iterate over each rib and grab the face of each rib to be partitioned
        for index2, (instanceName, instanceObject) in enumerate(instancesDictionary.items()):

            # Get the face on the rib to partition
            a = mdb.models['Model-1'].rootAssembly
            f = a.instances[instanceName].faces[index1]

            # Store the face in the array
            ribFaces[index2] = face

        # Select the datum plane for the current spar
        datumID = a.features[datumPlaneName].id
        datumPlane = a.datums[datumID]
        
        # Partition each rib for the current spar
        a.PartitionFaceByDatumPlane(datumPlane=datumPlane, faces=ribFaces)
'''

def generateSpars(sparDictionary, wingDictionary):

    print('Creating the spars from the skin')

    # Extract the data from the dictionaries
    sparLocations = sparDictionary['sparLocations']
    material = sparDictionary['material']
    thickness = sparDictionary['thickness']
    rootChord = wingDictionary['rootChord']
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']

    # Copy the skin part and create another part called Spars
    p = mdb.models['Model-1'].Part(name='Spars', 
    objectToCopy=mdb.models['Model-1'].parts['Skin'])

    # Create a reference frame at the origin
    p = mdb.models['Model-1'].parts['Spars']
    p.DatumCsysByThreePoints(name='referenceFrame', coordSysType=CARTESIAN, 
    origin=(0.0, 0.0, 0.0), line1=(1.0, 0.0, 0.0), line2=(0.0, 1.0, 0.0), 
    isDependent=False)

    # Define refFrame to be the coordinate system datum object
    desiredID = p.features['referenceFrame'].id
    refFrame = p.datums[desiredID]

    # Create a helper datum point above the origin and rename it
    p.DatumPointByCoordinate(coords=(0.0, 1.0, 0.0))
    mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum pt-1', 
    toName='HelperDatumPoint')

    # Initialize an array to store the names of the spar datum planes
    sparDatumPlanes = [None] * len(sparLocations)

    # For each spar location, create a datum plane that cuts through the skin at the spar location
    for index, sparLocation in enumerate(sparLocations):

        helperPlaneName = 'helperPlane' + str(index + 1)
        helperCoordinateSystemName = 'sparReferenceFrame' + str(index + 1)
        sparDatumPlaneName = 'rotatedPlane' + str(index + 1)
        sparDatumPlanes[index] = sparDatumPlaneName
        sparLocationInches = (sparLocation*rootChord)
        sparSweepAngle = computeSweepAngleAt(sparLocation, 0, leadingEdgeSweep, wingDictionary)

        # Create a helper datum plane and coordinate system at the point on root rib where the spar will be.
        p = mdb.models['Model-1'].parts['Spars']
        p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=sparLocationInches)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum plane-1', toName=helperPlaneName)
        p.DatumCsysByOffset(datumCoordSys=refFrame, name=helperCoordinateSystemName, coordSysType=CARTESIAN, vector=(sparLocationInches, 0.0, 0.0))

        # Create a datum plane and rotate it by the sweep angle so it represents the spar
        helperPlaneID = p.features[helperPlaneName].id
        helperPlane = p.datums[helperPlaneID]
        helperCoordinateSystemID = p.features[helperCoordinateSystemName].id
        helperCoordinateSystem = p.datums[helperCoordinateSystemID]
        p.DatumPlaneByRotation(plane=helperPlane, axis=helperCoordinateSystem.axis2, angle=sparSweepAngle)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum plane-1', toName=sparDatumPlaneName)

        '''
        ##### The issue with the following code is that I am rotating and THEN translating, but this is NOT the same as translating and then rotating. 
        # I need to translate to the point along the chord, and then rotate the datum planes.
        # Name the current datum axis and the datum plane
        axisName = 'spar' + str(index + 1) + 'DatumAxis'
        helperDatumPlaneName = 'spar' + str(index + 1) + 'HelperPlane'
        sparDatumPlaneName = 'spar' + str(index + 1) + 'DatumPlane'
        sparDatumPlanes[index] = sparDatumPlaneName

        # Create a helper datum axis representing the spar's direction from the root rib to the tip rib
        # Since each spar is at a different point along the chord, it will follow a unique angle down the wing spans
        sparSweepAngle = computeSweepAngleAt(sparLocation, 0, leadingEdgeSweep)
        p.DatumAxisByRotation(line=refFrame.axis3, axis=refFrame.axis2, angle=sparSweepAngle)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum axis-1', 
            toName=axisName)

        # Create a helper plane using the datum axis and the helper point above the origin
        lineID = p.features[axisName].id
        pointID = p.features['HelperDatumPoint'].id
        line = p.datums[lineID]
        point = p.datums[pointID]
        p.DatumPlaneByLinePoint(line=line, point=point)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum plane-1', 
            toName=helperDatumPlaneName)

        # Offset a datum plane to the point along the ribs where the spar should be
        ####### TODO: THE LOCATIONS OF THE DATUM PLANES ARE OFF FOR SOME REASON. sparLocationInches is correct, but when abaqus offsets the datum planes they're not in the right spot.
        # The (-7.4) is a temporary fix
        sparLocationInches = (sparLocation*rootChord)*(0.8)
        print(sparLocation, sparSweepAngle, rootChord, sparLocationInches)
        helperPlaneID = p.features[helperDatumPlaneName].id
        helperPlane = p.datums[helperPlaneID]
        p.DatumPlaneByOffset(plane=helperPlane, flip=SIDE2, offset=sparLocationInches)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Datum plane-1', 
            toName=sparDatumPlaneName)
        '''

    # Collect all the faces to be partitioned. This will be done by just selecting all the faces on the skin
    facesToBePartitioned = p.faces
    partitionFaceNames = [None] * len(sparLocations)

    # Iterate over each datum plane representing a spar, and partition the skin at the datum plane
    for index, datumPlaneName in enumerate(sparDatumPlanes):

        # Select the datum plane for the current spar
        datumID = p.features[datumPlaneName].id
        datumPlane = p.datums[datumID]
        
        # Partition each face on the skin for the current spar datum plane
        p.PartitionFaceByDatumPlane(datumPlane=datumPlane, faces=facesToBePartitioned)

        # Name the partition face
        partitionName = 'sparPartitionFace' + str(index+1)
        partitionFaceNames[index] = partitionName
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Partition face-1', toName=partitionName)
        
    # Iterate over each partition that was made and loft the top and the bottom edge together
    for index, partition in enumerate(partitionFaceNames):
        # Get the edges created from this partition
        p = mdb.models['Model-1'].parts['Spars']
        allEdges = p.edges
        edgesFromPartition = list(filter(lambda edge: edge.featureName == partition, allEdges))
        
        # Get a point on each edge
        edge1Coordinates = edgesFromPartition[0].pointOn[0]
        edge2Coordinates = edgesFromPartition[1].pointOn[0]

        # Loft the edges together
        # It's probably more efficient to just pass the edge into this function, but for some reason abaqus doesn't like it, so just stick with passing coordinates into findAt().
        p.ShellLoft(loftsections=((allEdges.findAt(coordinates=edge1Coordinates), ), (allEdges.findAt(coordinates=edge2Coordinates),)), startCondition=NONE, endCondition=NONE)

        # Rename the loft that was created
        loftName = 'sparLoft' + str(index + 1)
        mdb.models['Model-1'].parts['Spars'].features.changeKey(fromName='Shell Loft-1', toName=loftName)

    '''
    # I couldn't find a pattern on how the partitioned edges are indexed in the edges array, so im going to hardcode the lofting in.
    # If the configuration changes or the lofting causes an error, loft manually in the GUI and copy and paste again.
    p = mdb.models['Model-1'].parts['Spars']
    e = p.edges
    xx
    # Loft the front spar
    p.ShellLoft(loftsections=((e.findAt(coordinates=(137.1995, 6.742342, 
    145.65011)), ), (e.findAt(coordinates=(301.163372, -3.778121, 
    436.950164)), )), startCondition=NONE, endCondition=NONE)

    # Loft the middle spar
    p.ShellLoft(loftsections=((e.findAt(coordinates=(204.647244, 7.490095, 
    145.65021)), ), (e.findAt(coordinates=(340.063857, -3.715727, 436.950577)), 
    )), startCondition=NONE, endCondition=NONE)

    # Loft the back spar
    p.ShellLoft(loftsections=((e.findAt(coordinates=(278.890601, 4.794142, 
    145.650184)), ), (e.findAt(coordinates=(379.933489, -1.249257, 
    436.949741)), )), startCondition=NONE, endCondition=NONE)
    '''
    
    # Create a list of all the faces on the skin and delete them so we end up with just the spars
    faces = p.sets['skinSet'].faces
    p.RemoveFaces(faceList=faces, deleteCells=False)

    # Create spar material
    materialName = material['material']
    materialDensity = material['density']
    materialModulus = material['modulusOfElasticity']
    materialPoissonsRatio = material['poissonsRatio']

    mdb.models['Model-1'].Material(name=materialName)
    mdb.models['Model-1'].materials[materialName].Density(table=((materialDensity, ), ))
    mdb.models['Model-1'].materials[materialName].Elastic(table=((materialModulus, 
        materialPoissonsRatio), ))

    # Create the spar section
    sectionName = 'sparSection'
    mdb.models['Model-1'].HomogeneousShellSection(name=sectionName, 
    preIntegrate=OFF, material=materialName, thicknessType=UNIFORM, 
    thickness=thickness, thicknessField='', nodalThicknessField='', 
    idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)

    # Get the spars's faces
    faces = p.faces

    # Assign the section to the spar
    region = p.Set(faces=faces, name='sparSet')
    p = mdb.models['Model-1'].parts['Spars']
    p.SectionAssignment(region=region, sectionName=sectionName, offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

    # Instance the spars
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts['Spars']
    a.Instance(name='Spars-1', part=p, dependent=ON)

def generateStringers(partName, stringerDictionary, wingDictionary):

    print('Creating the stringers')

    # Game plan: 
    # 1) Partition the skin longitudinally like how you did for the spars and partition along those planes
    # 2) Create a horizontal plane at some height away from the top of the skin
    # 3) Loft the longitudinal partitions from (1) down to the plane created in (2). 
    # 4) Ideally we would be able to control the height of the stringers, see if you can figure that out

    # Extract the data needed from the dictionary
    chordLocations = stringerDictionary['chordLocations']
    thickness = stringerDictionary['thickness']
    height = stringerDictionary['height']
    material = stringerDictionary['material']
    materialName = material['material']
    materialDensity = material['density']
    materialModulus = material['modulusOfElasticity']
    materialPoissonsRatio = material['poissonsRatio']
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']
    rootChord = wingDictionary['rootChord']

    # Copy the skin part and create another part for the stringers
    p = mdb.models['Model-1'].Part(name=partName, 
    objectToCopy=mdb.models['Model-1'].parts['Skin'])

    # Create a reference frame at the origin
    p = mdb.models['Model-1'].parts[partName]
    p.DatumCsysByThreePoints(name='referenceFrame', coordSysType=CARTESIAN, 
    origin=(0.0, 0.0, 0.0), line1=(1.0, 0.0, 0.0), line2=(0.0, 1.0, 0.0), 
    isDependent=False)

    # Create a helper datum point above the origin and rename it
    p.DatumPointByCoordinate(coords=(0.0, 1.0, 0.0))
    mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum pt-1', 
    toName='HelperDatumPoint')

    # Get the properties of the reference frame
    desiredID = p.features['referenceFrame'].id
    refFrame = p.datums[desiredID]

    # Initialize an array to store the names of the stringer datum planes
    datumPlanes = [None] * len(chordLocations)

    # For each stringer location, create a datum plane that cuts through the skin at the stringer location
    for index, chordLocation in enumerate(chordLocations):
        '''
        ### THIS IS OLD CODE. It works but causes the stringers to be in the wrong place because it rotates and then translates
        ### It's supposed to translate then rotate
        # Name the current datum axis and the datum plane
        axisName = 'stringer' + str(index + 1) + 'DatumAxis'
        helperDatumPlaneName = 'stringer' + str(index + 1) + 'HelperPlane'
        datumPlaneName = 'stringer' + str(index + 1) + 'DatumPlane'
        datumPlanes[index] = datumPlaneName

        # Create a helper datum axis representing the stringer's direction from the root rib to the tip rib
        # Since each stringer is at a different point along the chord, it will follow a unique angle down the wing span
        sweepAngle = computeSweepAngleAt(chordLocation, 0, leadingEdgeSweep, wingDictionary)
        p.DatumAxisByRotation(line=refFrame.axis3, axis=refFrame.axis2, angle=sweepAngle)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum axis-1', 
            toName=axisName)

        # Create a helper plane using the datum axis and the helper point above the origin
        lineID = p.features[axisName].id
        pointID = p.features['HelperDatumPoint'].id
        line = p.datums[lineID]
        point = p.datums[pointID]
        p.DatumPlaneByLinePoint(line=line, point=point)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum plane-1', 
            toName=helperDatumPlaneName)

        # Offset a datum plane to the point along the ribs where the stringer should be
        ####### TODO: THE LOCATIONS OF THE DATUM PLANES ARE OFF FOR SOME REASON. locationInches is correct, but when abaqus offsets the datum planes they're not in the right spot.
        # The (-7.4) is a temporary fix
        locationInches = chordLocation*rootChord
        helperPlaneID = p.features[helperDatumPlaneName].id
        helperPlane = p.datums[helperPlaneID]
        p.DatumPlaneByOffset(plane=helperPlane, flip=SIDE2, offset=locationInches)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum plane-1', 
            toName=datumPlaneName)
        '''
        
        ### The code below translates then rotates
        helperPlaneName = 'stringer' + str(index + 1) + 'helperPlane'
        helperCoordinateSystemName = 'stringer' + str(index + 1) + 'ReferenceFrame'
        datumPlaneName = 'stringer' + str(index + 1) + 'DatumPlane'
        datumPlanes[index] = datumPlaneName
        locationInches = (chordLocation*rootChord)
        sweepAngle = computeSweepAngleAt(chordLocation, 0, leadingEdgeSweep, wingDictionary)

        # Create a helper datum plane and coordinate system at the point on root rib where the stringer will be.
        p = mdb.models['Model-1'].parts[partName]
        p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=locationInches)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum plane-1', toName=helperPlaneName)
        p.DatumCsysByOffset(datumCoordSys=refFrame, name=helperCoordinateSystemName, coordSysType=CARTESIAN, vector=(locationInches, 0.0, 0.0))

        # Create a datum plane and rotate it by the sweep angle so it represents the stringer
        helperPlaneID = p.features[helperPlaneName].id
        helperPlane = p.datums[helperPlaneID]
        helperCoordinateSystemID = p.features[helperCoordinateSystemName].id
        helperCoordinateSystem = p.datums[helperCoordinateSystemID]
        p.DatumPlaneByRotation(plane=helperPlane, axis=helperCoordinateSystem.axis2, angle=sweepAngle)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Datum plane-1', toName=datumPlaneName)

    # Get the skin faces so they can be partitioned, and create an array that will store the names of partitions.
    facesToBePartitioned = p.faces
    partitionFaceNames = [None] * len(chordLocations)

    # Iterate over each datum plane representing a stringer, and partition the skin at the datum plane
    for index, datumPlaneName in enumerate(datumPlanes):

        # Select the datum plane for the current stringer
        datumID = p.features[datumPlaneName].id
        datumPlane = p.datums[datumID]
        
        # Partition each face on the skin for the current stringer datum plane
        p.PartitionFaceByDatumPlane(datumPlane=datumPlane, faces=facesToBePartitioned)

        # Name the partition face
        partitionName = 'stringerPartitionFace' + str(index+1)
        partitionFaceNames[index] = partitionName
        mdb.models['Model-1'].parts['Stringers'].features.changeKey(fromName='Partition face-1', toName=partitionName)

    # Iterate over each partition that was made and loft the top and the bottom edge together
    for index, partition in enumerate(partitionFaceNames):

        # Get the edges created from this partition
        p = mdb.models['Model-1'].parts[partName]
        allEdges = p.edges
        edgesFromPartition = list(filter(lambda edge: edge.featureName == partition, allEdges))

        # Get a point on each edge
        edge1Coordinates = edgesFromPartition[0].pointOn[0]
        edge2Coordinates = edgesFromPartition[1].pointOn[0]

        # Loft the edges together
        # It's probably more efficient to just pass the edge into this function, but for some reason abaqus doesn't like it, so just stick with passing coordinates into findAt().
        p.ShellLoft(loftsections=((allEdges.findAt(coordinates=edge1Coordinates), ), (allEdges.findAt(coordinates=edge2Coordinates),)), startCondition=NONE, endCondition=NONE)

        # Rename the loft that was created
        loftName = 'stringerLoft' + str(index + 1)
        mdb.models['Model-1'].parts[partName].features.changeKey(fromName='Shell Loft-1', toName=loftName)
        
        # Perform two partitions, first the top stringers then the bottom stringers
        for index1 in xrange(0, 2):

            # Find the faces and edges of the loft
            # loftEdges initially (before any partitions) has the form: [topEdge, loftEdgeAtTip, bottomEdge, loftEdgeAtRoot]
            loftFaces = list(filter(lambda face: face.featureName == loftName, p.faces))
            loftEdges = list(filter(lambda edge: edge.featureName == loftName, p.edges))

            # If partitioning the top stringers, use a certain set of faces and edges
            if index1 == 0:
                loftFace = loftFaces[0]
                loftEdgeAtTip = loftEdges[1]
                loftEdgeAtRoot = loftEdges[3]

            # If partitioning the bottom stringers, the faces and edges change so get the right ones
            if index1 == 1:
                loftFace = loftFaces[0]
                loftEdgeAtTip = loftEdges[0]
                loftEdgeAtRoot = loftEdges[2]
             
            # Calculate the height of both edges
            tipEdgeHeight = loftEdgeAtTip.getSize(printResults = 0)
            rootEdgeHeight = loftEdgeAtRoot.getSize(printResults = 0)

            # Calculate the parameter needed to partition for the desired stringer height
            # This is the parameter for the top stringers
            tipParameter = height#/tipEdgeHeight
            rootParameter = height#/rootEdgeHeight

            # If partitioning for the bottom stringers, we want the partition at the bottom of the edge.
            # The parameter specifies the distance from the top of the skin down the length of the edge.
            if index1 == 1:
                tipParameter = 1 - tipParameter
                rootParameter = 1 - rootParameter

            # Partition the loft
            p.PartitionFaceByCurvedPathEdgeParams(face=loftFace, edge1=loftEdgeAtRoot, edge2=loftEdgeAtTip, 
            parameter1=rootParameter, parameter2=tipParameter)

        # Delete the middle face so we're left with just the top and bottom stringers. The middle face is the first face in the list loftFaces
        loftFaces = list(filter(lambda face: face.featureName == loftName, p.faces))
        del loftFaces[0]
        del loftFaces[1]
        p.RemoveFaces(faceList = loftFaces, deleteCells=False)
        
    # Create a list of all the faces on the skin and delete them so we end up with just the stringers
    faces = p.sets['skinSet'].faces
    p.RemoveFaces(faceList=faces, deleteCells=False)

    # Create material
    materialName = material['material']
    materialDensity = material['density']
    materialModulus = material['modulusOfElasticity']
    materialPoissonsRatio = material['poissonsRatio']

    mdb.models['Model-1'].Material(name=materialName)
    mdb.models['Model-1'].materials[materialName].Density(table=((materialDensity, ), ))
    mdb.models['Model-1'].materials[materialName].Elastic(table=((materialModulus, 
        materialPoissonsRatio), ))

    # Create the section
    sectionName = 'stringerSection'
    mdb.models['Model-1'].HomogeneousShellSection(name=sectionName, 
    preIntegrate=OFF, material=materialName, thicknessType=UNIFORM, 
    thickness=thickness, thicknessField='', nodalThicknessField='', 
    idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
    thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
    integrationRule=SIMPSON, numIntPts=5)

    # Get the parts faces
    p = mdb.models['Model-1'].parts[partName]
    faces = p.faces

    # Assign the section to the part
    region = p.Set(faces=faces, name='stringerSet')
    p = mdb.models['Model-1'].parts[partName]
    p.SectionAssignment(region=region, sectionName=sectionName, offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

    # Instance the part
    instanceName = partName + '-1'
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts[partName]
    a.Instance(name=instanceName, part=p, dependent=ON)

def mergeWing(wingDictionary):

    print('Merging the wing')

    # Extract the variables from the dictionary
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']

    # Merge the wing together
    a = mdb.models['Model-1'].rootAssembly
    a.InstanceFromBooleanMerge(name='MergedWing', instances=(
    a.instances['AllRibs-1'], a.instances['Skin-1'], a.instances['Spars-1'], a.instances['Stringers-1'], ), 
    originalInstances=SUPPRESS, domain=GEOMETRY)

    # Create a reference frame at the origin
    p = mdb.models['Model-1'].parts['MergedWing']
    p.DatumCsysByThreePoints(name='referenceFrame', coordSysType=CARTESIAN, 
    origin=(0.0, 0.0, 0.0), line1=(1.0, 0.0, 0.0), line2=(0.0, 1.0, 0.0), 
    isDependent=False)

    # Define refFrame to be the coordinate system datum object
    refFrameID = p.features['referenceFrame'].id
    refFrame = p.datums[refFrameID]

    # Create a helper datum point above the origin and rename it
    p.DatumPointByCoordinate(coords=(0.0, 1.0, 0.0))
    mdb.models['Model-1'].parts['MergedWing'].features.changeKey(fromName='Datum pt-1', 
    toName='HelperDatumPoint')

    # Create a coordinate system at quarter chord, so we can use this coordinate system for the axis when defining a load later
    # p.DatumCsysByOffset(datumCoordSys=refFrame, name=helperCoordinateSystemName, coordSysType=CARTESIAN, vector=(sparLocationInches, 0.0, 0.0))

    # Create a datum axis centered at half the root chord, and points along the wingspan at the half chord sweep angle
    p = mdb.models['Model-1'].parts['MergedWing']
    d = p.datums
    halfChordSweepAngle = computeSweepAngleAt(0.5, 0, leadingEdgeSweep, wingDictionary)
    p.DatumAxisByRotation(line=refFrame.axis3, axis=refFrame.axis2, angle=halfChordSweepAngle)
    mdb.models['Model-1'].parts['MergedWing'].features.changeKey(fromName='Datum axis-1', toName='halfChordDatumAxis')

    # Create a reference frame at the origin pointing along the half chord sweep angle
    refFrameID = p.features['referenceFrame'].id
    refFrame = p.datums[refFrameID]
    halfChordAxisID = p.features['halfChordDatumAxis'].id
    halfChordAxis = p.datums[halfChordAxisID]
    p.DatumCsysByTwoLines(CARTESIAN, line1=halfChordAxis, line2=refFrame.axis2, 
        name='wingReferenceFrame')

def rotateWing(chordLocationToRotate, subsonicLeadingEdgeSweep, supersonicLeadingEdgeSweep):

    print('Rotating the wing for the supersonic case')

    locationToRotateInches = rootChord*chordLocationToRotate
    subsonicSweepAngle = computeSweepAngleAt(chordLocationToRotate, 0, subsonicLeadingEdgeSweep)
    supersonicSweepAngle = computeSweepAngleAt(chordLocationToRotate, 0, supersonicLeadingEdgeSweep)
    rotationAngle = abs(supersonicSweepAngle - subsonicSweepAngle) 

    a = mdb.models['Model-1'].rootAssembly
    a.rotate(instanceList=('MergedWing-1', ), axisPoint=(locationToRotateInches, 0.0, 0.0), axisDirection=(0.0, 1.0, 0.0), angle=rotationAngle)

def generateMeshOnInstance(meshDictionary):

    # Extract the data from the meshDictionary
    instanceName = meshDictionary['instanceName']
    seedSize = meshDictionary['seedSize']
    order = meshDictionary['order']
    technique = meshDictionary['technique']
 
    # Define the root assembly module
    a = mdb.models['Model-1'].rootAssembly

    # Seed the instance
    partInstances = (a.instances[instanceName], )
    a.seedPartInstance(regions=partInstances, size=seedSize, deviationFactor=0.1, minSizeFactor=0.1)

    # Define the mesh controls for the ribs
    faces = a.instances[instanceName].faces
    a.setMeshControls(regions=faces, elemShape=QUAD_DOMINATED, technique=technique)

    # Define the proper element codes based on if it's linear or quadratic
    elementCodes = [None, None]
    secondOrderAccuracy = None

    if order == 'linear':
        elementCodes[0] = S4R
        elementCodes[1] = S3
        secondOrderAccuracy = OFF
    elif order == 'quadratic':
        elementCodes[0] = S8R
        elementCodes[1] = STRI65
        secondOrderAccuracy = ON

    # Define the element type for the ribs (linear)
    elemType1 = mesh.ElemType(elemCode=elementCodes[0], elemLibrary=STANDARD, secondOrderAccuracy=secondOrderAccuracy, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=elementCodes[1], elemLibrary=STANDARD)
    a = mdb.models['Model-1'].rootAssembly
    faces = a.instances['AllRibs-1'].faces
    pickedRegions = (faces, )
    a.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))

    # Mesh the instance
    a.generateMesh(regions=partInstances)

def generateMesh(seedSize):

    print('Generating the mesh')

    # Seed the part
    p = mdb.models['Model-1'].parts['MergedWing']
    p.seedPart(size=seedSize, deviationFactor=0.1, minSizeFactor=0.1)

    # Define the element controls
    p = mdb.models['Model-1'].parts['MergedWing']
    faces = p.faces
    faces1=faces.getByBoundingBox(-500, 500,-500, 500, -500, 500)
    p.setMeshControls(regions=faces1, elemShape=QUAD, technique=STRUCTURED)

    # Define the element types
    elemType1 = mesh.ElemType(elemCode=S4R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=S3, elemLibrary=STANDARD)
    p = mdb.models['Model-1'].parts['MergedWing']
    faces = p.faces
    pickedRegions = (faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))

    # Mesh the part
    p = mdb.models['Model-1'].parts['MergedWing']
    p.generateMesh()

def generateLoadsAndBCs(loadDictionary, numberOfRibs, sparLocations, wingDictionary):

    print('Assigning loads and boundary conditions')

    # Extract data from the dictionaries
    wingSpan = wingDictionary['wingSpan']
    leadingEdgeSweep = wingDictionary['leadingEdgeSweep']
    highLoadFunction = loadDictionary['highLoadFunction']
    lowLoadFunction = loadDictionary['lowLoadFunction']
    loadFactor = loadDictionary['loadFactor']
    pressureMagnitude = loadDictionary['pressureMagnitude']

    # Create the tipnode set
    a = mdb.models['Model-1'].rootAssembly
    wing = a.instances['MergedWing-1']
    tipRibSetName = 'MergedWing-1.rib' + str(numberOfRibs) + 'Set'
    tipRibFaces = a.sets[tipRibSetName].faces                                     # Get all the faces on the tip rib
    tipRibEdgeIDs = [ face.getEdges() for face in tipRibFaces ]                   # Get all the edge IDs on the tip rib
    flatListEdgeIDs = [edgeID for sublist in tipRibEdgeIDs for edgeID in sublist] # Convert the list of lists to just one list
    allTipRibEdges = [ wing.edges[edgeID] for edgeID in flatListEdgeIDs ]         # Get all the edge objects on the tip rib
    edgeCoordinates = allTipRibEdges[0].pointOn[0]                                # Get a point on the first edge in the list
    edge = wing.edges.findAt( ((edgeCoordinates[0], edgeCoordinates[1], edgeCoordinates[2]), ))
    a.Set(edges=edge, name='TIPNODE')                                             # Define the first edge in the set to be the TIPNODE set
    '''
    # Define a surface set so we can check for buckling on the skin
    wing = a.instances['MergedWing-1']
    skinSetName = 'MergedWing-1.skinSet'
    skinFaces = a.sets[skinSetName].faces                                                      # Get all the faces on the skin
    faceRegion = []
    for index, face in enumerate(skinFaces):
        coordinates = face.pointOn[0]

        # Get all the faces behind the aft spar, close to the root rib, and on the top of the skin
        if ( (coordinates[0] > sparLocations[-1]) and (coordinates[2] < 0.2*wingSpan) and (coordinates[1] > 0) ):
            faceRegion.append(face)
    print('HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE')
    print(faceRegion)
    xxx
    edgeCoordinates = allTipRibEdges[0].pointOn[0]                                # Get a point on the first edge in the list
    edge = wing.edges.findAt( ((edgeCoordinates[0], edgeCoordinates[1], edgeCoordinates[2]), ))
    a.Set(edges=edge, name='TIPNODE')                                             # Define the first edge in the set to be the TIPNODE set
    
    a = mdb.models['Model-1'].rootAssembly
    s1 = a.instances['MergedWing-1'].faces
    side1Faces1 = s1.findAt(((295.440874, 3.428336, 129.462689), ))
    print(side1Faces1[0])
    print(side1Faces1.getNodes())
    #a.Surface(side1Faces=side1Faces1, name='Surf-3')
    xxx
    # edgesWithNodes = list(filter(lambda edge: not edge.getNodes() == None, flatListEdgeIDs))  # Get all the edges on the face that have nodes
    # nodes = edgesWithNodes[0].getNodes()                                                      # Get the node IDs on the first edge that has nodes
    # nodeForAbaqusSyntax = [wing.nodes[ (nodeLabel) : (nodeLabel + 1) ]]   # Get the node object using the node's label because this is how abaqus likes it
    '''
    # Encaster the wing at the root rib
    a = mdb.models['Model-1'].rootAssembly
    region = a.instances['MergedWing-1'].sets['rib1Set']
    mdb.models['Model-1'].EncastreBC(name='FixedBC', createStepName='Step-1', region=region, localCsys=None)

    # Rename first datum frame to globalReferenceFrame
    mdb.models['Model-1'].rootAssembly.features.changeKey(fromName='Datum csys-1', toName='globalReferenceFrame')

    # Create a reference frame at the origin pointing along the wingspan
    p = mdb.models['Model-1'].parts['MergedWing']
    p.features['wingReferenceFrame'].id
    wingRefFrameID = p.features['wingReferenceFrame'].id#mbd.models['Model-1'].rootAssembly.instances['MergedWing-1'].datums
    wingRefFrame = mdb.models['Model-1'].rootAssembly.instances['MergedWing-1'].datums[wingRefFrameID]

    #a = mdb.models['Model-1'].rootAssembly
    #globalFrameID = a.features['globalReferenceFrame'].id
    #globalFrame = a.datums[globalFrameID]
    
    # Create the elliptical wing loading function for both load regions
    mdb.models['Model-1'].ExpressionField(name='HighLoadField', localCsys=None, description='', expression=highLoadFunction)
    mdb.models['Model-1'].ExpressionField(name='LowLoadField', localCsys=None, description='', expression=lowLoadFunction)

    # Get all the faces on the top of the skin before the faces at the tip (the tip faces for some reason cause division by zero error when defining the analytical field)
    locationOfSecondToLastRib = (wingSpan/2) - (wingSpan/2)/(numberOfRibs - 1)
    a = mdb.models['Model-1'].rootAssembly
    skinFaces = a.sets['MergedWing-1.skinSet'].faces
    facesBeforeTipFaces = list(filter(lambda face: face.pointOn[0][2] < locationOfSecondToLastRib, skinFaces)) # Get the faces before the z coordinate of the second to last rib
    facesOnTopOfSkin = list(filter(lambda face: face.pointOn[0][1] > 0, facesBeforeTipFaces))                  # Get the top faces (faces with y coordinates > 0)
    region = a.Surface(side1Faces=part.FaceArray(facesOnTopOfSkin), name='EllipticalLoadingSurface')    # Define a region consisting of all skin faces on top and before tip faces

    # Separate the faces on top of the skin based on if they are before or after the quarter chord
    facesBeforeQuarterChord = []
    facesAfterQuarterChord = []
    for face in facesOnTopOfSkin:

        # For the z value of the face, see if it's x value is before quarter chord
        if face.pointOn[0][0] < computeSweptXValue(face.pointOn[0][2], 0.25, wingDictionary):
            facesBeforeQuarterChord.append(face)
        else:

            # Get the rest of the faces that are after the quarter chord, on top of the skin, and before the tip faces.
            facesAfterQuarterChord.append(face)


    # Define the regions to take loading
    highLoadRegion = a.Surface(side1Faces=part.FaceArray(facesBeforeQuarterChord), name='HighMagnitudeLoadingSurface')
    lowLoadRegion = a.Surface(side1Faces=part.FaceArray(facesAfterQuarterChord), name='LowMagnitudeLoadingSurface')

    # Apply the elliptical load to the faces we just obtained. 
    # Make sure to change faceOnTopOfSkin from a list to a FaceArray. See this link: 
    # https://stackoverflow.com/questions/52573521/how-to-create-set-from-list-of-sets-in-python
    mdb.models['Model-1'].Pressure(name='highLoad', createStepName='Step-1', 
        region=highLoadRegion, distributionType=FIELD, field='HighLoadField', 
        magnitude=loadFactor, amplitude=UNSET)
    mdb.models['Model-1'].Pressure(name='lowLoad', createStepName='Step-1', 
        region=lowLoadRegion, distributionType=FIELD, field='LowLoadField', 
        magnitude=loadFactor, amplitude=UNSET)

    # Get the tip faces
    tipFaces = list(filter(lambda face: face.pointOn[0][2] >= locationOfSecondToLastRib, skinFaces)) # Get the faces after the z coordinate of the second to last rib
    tipFacesAboveSkin = list(filter(lambda face: face.pointOn[0][1] > 0, tipFaces))                  # Get the top faces (faces with y coordinates > 0)

    # Apply a pressure force to the tip faces
    region = a.Surface(side1Faces=part.FaceArray(tipFacesAboveSkin), name='tipFaces')
    mdb.models['Model-1'].Pressure(name='pressureLoad', createStepName='Step-1', 
    region=region, distributionType=UNIFORM, field='', magnitude=pressureMagnitude, 
    amplitude=UNSET)

def createAndRunJob():

    print('Creating the job')

    # Create the job
    ModelName = 'Model-1'
    mdb.Job(name=ModelName, model=ModelName, description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)
    job=mdb.jobs[ModelName]

    # delete lock file, which for some reason tends to hang around, if it exists
    if os.access('%s.lck'%ModelName,os.F_OK):
        os.remove('%s.lck'%ModelName)
        
    # Run the job
    job.submit()
    print('Running the job')      
    job.waitForCompletion()
    print('Completed job')

def main(designVariablesDictionary, loadCase, loadFactor, seedSize):

    #####################################
    ### Define variables              ###
    #####################################

    ##### All units are in lbf, in, and psi #####
    numberOfRibs = designVariablesDictionary['numberOfRibs']
    skinThickness = designVariablesDictionary['skinThickness']
    sparThickness = designVariablesDictionary['sparThickness']
    ribThickness = designVariablesDictionary['ribThickness']
    circleDiameter = designVariablesDictionary['circleDiameter']

    ribData = {
        'numberOfRibs': numberOfRibs,
        'thickness': ribThickness,
        'circleDiameter': circleDiameter,
        'material': generateMaterialDictionary('Aluminum7075', 10400000, .102, 0.33),
        'airfoilCoordinates': 'C:\\temp\\hunterNewDirectory\\CoordinateData\\normalizedCoordinates.csv'
    }

    wingData = {
        'aspectRatio': 6,
        'taperRatio': 0.25,
        'wingArea': (1570.0*144.0),
        'leadingEdgeSweep': radians(32.0),
        'wingSpan': (97.1*12.0),
        'rootChord': (25.9*12.0),
        'tipChord': (6.5*12.0)
    }

    sparData = {
        'sparLocations': [0.155, 0.45, 0.745],
        'material': generateMaterialDictionary('Aluminum7075', 10400000, .102, 0.33),
        'thickness': sparThickness
    }

    stringerLocations = computeStringerLocations(10, sparData['sparLocations'])
    stringerData = {
        'chordLocations': stringerLocations,
        'height': 0.15,
        'material': generateMaterialDictionary('Aluminum7075', 10400000, .102, 0.33),
        'thickness': 0.04
    }

    skinData = {
        'material': generateCompositeDictionary('Composite', .10, 5, skinThickness, [20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0]),
        'thickness': skinThickness
    }

    # Define the weight for the loading
    if loadCase == 'takeoff':
        planeWeight = 138700.0 # lbf
        maxLoad = -0.30

    if loadCase == 'cruise':
        planeWeight = 131000.0 # lbf
        maxLoad = -0.375

    # Define the loads on the wing
    locationOfSecondToLastRib = (wingData['wingSpan']/2) - (wingData['wingSpan']/2)/(ribData['numberOfRibs'] - 1)
    xValue = (wingData['wingSpan']/2.0) - (0.25)*(wingData['wingSpan']/2.0)/(float(numberOfRibs) - 1) # Get the x value a quarter step back from the tip rib
    pressureLoad = maxLoad*0.05                                     # *sqrt( 1.0 - pow((xValue)/(wingData['wingSpan']/2.0), 2) )
    ellipticalLoad = str(maxLoad) + '*sqrt ( 1 - pow(Z/570,2) )'    # *(1 - X/350)' # + str(locationOfSecondToLastRib) + ')'

    loadData = {
        'highLoadFunction': ellipticalLoad,
        'lowLoadFunction': ellipticalLoad + '*' + str(0.1),
        'pressureMagnitude': pressureLoad,
        'loadFactor': loadFactor
    }

    #####################################
    ### Generation of SOLID FEA Model ###
    #####################################

    # Erase any existing contents and open a new Mdb file
    Mdb()

    # This allows ABAQUS to use findAt() instead of mask
    session.journalOptions.setValues(replayGeometry = COORDINATE, recoverGeometry = COORDINATE)

    # Create the rib parts and assign materials, sections, and create instances
    generateRibs(ribData, sparData, wingData)

    # Create the skin
    generateSkin(ribData, skinData, wingData['rootChord'])

    # Partition the skin where the spars are going to be and generate the spars by lofting
    generateSpars(sparData, wingData)
    
    # Create the stringers on the skin
    generateStringers('Stringers', stringerData, wingData)

    # Merge all the parts together so they form one rigid body
    mergeWing(wingData)

    # Generate the meshes
    generateMesh(seedSize)

    # Create a step
    mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

    # Create loads and boundary conditions
    generateLoadsAndBCs(loadData, ribData['numberOfRibs'], sparData['sparLocations'], wingData)

    # Create and run a job
    createAndRunJob()

    # Extract the results
    massProperties = mdb.models['Model-1'].rootAssembly.getMassProperties()
    massRibsProperties = mdb.models['Model-1'].parts['AllRibs'].getMassProperties()
    stress, deflection = getResults('Model-1')

    results = {
        'numberOfRibs': numberOfRibs,
        'ribThickness': ribThickness,
        'circleDiameter': circleDiameter,
        'sparThickness': sparThickness,
        'skinThickness': skinThickness,
        'totalMass': massProperties['mass'],
        'deflection': deflection,
        'stress': stress,
        'massRibs': massRibsProperties['mass']
    }

    return results

# ##### Example of how to run the code #####
# # Create a dictionary for the design variables
# designVariablesDictionary = {
#     'numberOfRibs': 5,
#     'skinThickness': 0.05,           # inches
#     'sparThickness': 0.75,           # inches
#     'ribThickness': 0.03,             # inches
#     'circleDiameter': 0.25	     # percentage of chord
# }

# loadFactor = 1.5
# seedSize = 10
# loadCondition = 'takeoff'
# results = main(designVariablesDictionary, loadCondition, loadFactor, seedSize)

'''
######## TODO ############
# 2. Figure out how to fix the division by zero error
# 3. [DONE] Get a DOE
# 4. Get a genetic algorithim
# 5. [DONE] Set up stringers
# 6. [DONE] Create a tip node set so post data can calculate deflection at the tip
# 7. [DONE] Automate the selection of surfaces for the load based, right now it uses find at
# 8. [DONE] Fix spar location issue
# 1. [DONE] Get the circles into the ribs
# 6. [DONE] Automate loads and boudary conditions
# 8. [DONE] automate spar lofting
# 9. Create a reference from down the length of the wingspan and use that for the loading
# 10. [DONE] It would be better if I partition all the ribs after generating the skin. Just create the datum planes when making the ribs, then after generating the skin create the partitions
# 11. [DONE] Partition ribs horizontally
# 12. Stringers?

# print(faces.getSize())      # THIS IS HOW TO GET THE AREA OF A FACE
# cae nogui = dummedForDom.py # Don't use the GUI to save time

### HOW THIS CODE CAN BE IMPROVED ###
# 1) OOP
# 2) The stringers and spars are generated essentially the same, so a functinon could be created called partitionSkinAlongWingspan() that could take
#    inputs of where the partitions are located, and the name of the part being created, then automate everything. Then inside the stringers and spar
#    functions this can be called for the list of locations, then all that has to be done inside the stringers and spar functions is to loft the edges.
# 3) Each function should take the model name and part name as an input, then it wouldn't have to be typed out all the time.
# 4) There are several pieces of logic that were used multiple times in almost identical ways, ideally for each piece of logic, this would become a function
#    so we're not just copying and pasting the logic in different places.

# # Example of how to run the code: 
#
# # Define the design variables
# designVariablesDictionary = {
#     'numberOfRibs': 7,
#     'skinThickness': 0.15, # inches
#     'sparThickness': 2.25, # inches
#     'ribThickness': 0.25,  # inches
#     'circleDiameter': 0.6  # percentage of chord
# }
#
# # Define the load cases and load factor
# loadCases = ['takeoff', 'supersonicCruise', 'landing']
# loadFactor = 1
# main(designVariablesDictionary, 'takeoff', 1)
'''