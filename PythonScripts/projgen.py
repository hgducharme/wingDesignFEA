#from buckle import buckle
from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import math
import random 
from LHS import LHS
from wingHunter import *
from Post_P_Script import getResults

nr_range = (7, 30)
tr_range = (0.03, 0.2) 
r_range = (0.25, 0.50)		#percentage 
ts_range = (0.5, 2.0)
t_range = (0.10, 0.5)

stringerDictionary = {
    ## TODO: Create an evenly divided set of points from the front spar location to the back spar, but don't let any points be where a spar is located, this will be the stringer locations
    'chordLocations': [0.16, 0.19, 0.24, 0.27, 0.31, 0.36, 0.39, 0.42, 0.47, 0.5, 0.55, 0.6, 0.65],
    'height': 0.2, # inch
    'material': None,
    'thickness': 0.04
}

ribMaterial = generateMaterialDictionary('Aluminum7075', 10400000, .102, 0.33)
sparMaterial = generateMaterialDictionary('Aluminum7075', 10400000, .102, 0.33)

stringerDictionary['material'] = generateMaterialDictionary('Aluminum2024', 1060000, .10, 0.30)
loadCase = 'takeoff'
if loadCase == 'takeoff':
    planeWeight = 138700.0                 # lbf

if loadCase == 'cruise':
    planeWeight = 131000.0                 # lbf


resfile1='mat3.txt' #necessary matrix file, sorted LHS file  

var=5 #nribs,tribs,rpercent,tspars,tskin
samp=100

runs=LHS(var,samp)



wdth, hgt =8, samp #h_min,h_max,w_min,w_max,t_min,t_max,tp_min,tp_max,ns_min,ns_max
mMatrix = [[0 for x in range(wdth)] for y in range(hgt)]

temp = [] #temporary array to be put in mMatrix later
#5 variables that can be interated, each need their own marker 
j=0
k=1
l=2
m=3
n=4
dr = 0

# for i in range(0,samp):
	# #Mdb()
	# pc=(runs[j],runs[k],runs[l],runs[m],runs[n])#randomization for modifiable variables 
	
	# numberOfRibs = int(math.floor(nr_range[1]-(nr_range[1]-nr_range[0])*pc[0]))
	# skinThickness = t_range[1]-(t_range[1]-t_range[0])*pc[1]            # in
	# sparThickness = ts_range[1]-(ts_range[1]-ts_range[0])*pc[2]            # in
	# ribThickness = tr_range[1]-(tr_range[1]-tr_range[0])*pc[3]               # in
	# circleDiameter =r_range[1]-(r_range[1]-r_range[0])*pc[4]				#percentage 
	# seedSize = 2
	# skinMaterialComposite = generateCompositeDictionary('Composite', .10, 5, skinThickness, [20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0])
	# # loadCase = 'takeoff'
	# loadFactor = 1.5
	# airfoilCoordinates = 'C:\\temp\\proj88\\CoordinateData\\normalizedCoordinates.csv'
	# aspectRatio = 6
	# taperRatio = 0.25
	# wingArea = feetToInches(1570.0*12.0)       # square in
	# leadingEdgeSweep = radians(32)         # rad
	# wingSpan = feetToInches(97.1)          # in
	# halfWingSpan = wingSpan/2.0            # in
	# rootChord = chord(0)                   # in
	# tipChord = chord(halfWingSpan)         # in

	# # Spar information ## TODO: This should be a dictionary that can be passed similar to the stringers
	# frontSparLocation = 0.155
	# backSparLocation = 0.695
	# middleSparLocation = (1.0/2.0)*(frontSparLocation + backSparLocation)
	# sparLocations = [0.155, 0.55, 0.795]
	# maxLoad = (0.5)*(planeWeight/wingArea) # psi
	# xValue = halfWingSpan - (0.25)*halfWingSpan/(float(numberOfRibs) - 1) # Get the x value a quarter step back from the tip rib
	# pressureLoad = -maxLoad*sqrt( 1.0 - pow((halfWingSpan - (0.25)*halfWingSpan/(float(numberOfRibs) - 1))/halfWingSpan, 2) )
	# ellipticalLoad = '-.30669674451*sqrt ( 1-pow(Z/570,2))'
	# locationOfSecondToLastRib = (wingSpan/2) - (wingSpan/2)/(numberOfRibs - 1)
	
	# Mdb()
	
	# # This allows ABAQUS to use findAt() instead of mask
	# session.journalOptions.setValues(replayGeometry = COORDINATE, recoverGeometry = COORDINATE)

	# locationOfSecondToLastRib = (wingSpan/2) - (wingSpan/2)/(numberOfRibs - 1)

	# # Create the rib parts and assign materials, sections, and create instances
	# generateRibs(numberOfRibs, ribMaterial, ribThickness, sparLocations, circleDiameter)

	# # Create the skin
	# generateSkin(numberOfRibs, skinMaterialComposite, skinThickness)

	# # Partition the ribs (delete the code in generateRibs())
	# #### TODO: AllRibs needs to use the newly partitioned ribs
	# partitionRibs(numberOfRibs)

	# # Partition the skin where the spars are going to be and generate the spars by lofting
	# generateSpars(sparLocations, sparMaterial, sparThickness)

	# # Create the stringers on the skin
	# generateStringers('Stringers', stringerDictionary)

	# # Merge all the parts together so they form one rigid body
	# mergeWing()

	# # Rotate the wing
	# #if loadCase == 'cruise':
	# #    rotateWing(0.33, 32, 60)

	# # Generate the meshes
	# generateMesh(seedSize)

	# # Create a step
	# mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

	# # Create loads and boundary conditions
	# generateLoadsAndBCs(ellipticalLoad, loadFactor, pressureLoad, numberOfRibs) #numberOfRibs

	# # Create and run a job
	# createAndRunJob()
	
	# ModelName='Model-1'
	# MassProperties=mdb.models['Model-1'].rootAssembly.getMassProperties()
	
	# mass=MassProperties['mass']
	
	# MassProperties2=mdb.models['Model-1'].parts['AllRibs'].getMassProperties()
	# massribs=MassProperties2['mass']
	# # stress=getResults(ModelName)
	# # deflection=getResults(ModelName)
	# d,f=getResults(ModelName)
	# stress=d
	# deflection=f
	
	
	# temp=[numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass,massribs]
	
	# if stress>73000.0 or deflection>60.0 : ##set failures apart 
		# temp[5]=10**10
	# mMatrix[dr] = temp #substitute 
	# dr = dr + 1 #next row
	# j=j+4
	# k=k+4
	# l=l+4
	# m=m+4
	# n=n+4


# mat = sorted(mMatrix, key=lambda row:row[5])
# dataFile = open(resfile1,'w')
# dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass,massribs\n')
# count = 0
# for i in mat: #transcript entire sorted matrix 
    # for cell in mat[count]:
        # dataFile.write("%s," % cell)
    # count = count + 1
    # dataFile.write('\n')
# dataFile.close()
mat=[[29, 0.0572, 0.4025, 0.785, 0.112, 3041.22559070404, 285.967152398968], [27, 0.1235, 0.2625, 0.605, 0.116, 3098.24240113936, 582.795783208471], [15, 0.0606, 0.47, 0.83, 0.124, 3141.20169375623, 156.574425079148], [19, 0.0555, 0.2675, 0.965, 0.112, 3232.37490140838, 184.903754263304], [8, 0.0895, 0.5, 0.635, 0.184, 3574.51798891484, 124.667367647419], [24, 0.1694, 0.49, 0.53, 0.156, 3610.25618845611, 694.248726447844], [23, 0.1694, 0.2825, 0.98, 0.104, 3650.37562740088, 680.937534145016], [9, 0.0402, 0.44, 1.115, 0.144, 3773.15112075205, 63.2709881221124], [22, 0.0487, 0.29, 0.95, 0.16, 3836.10572198656, 187.22120000498], [10, 0.1507, 0.405, 1.325, 0.124, 4054.11904760517, 263.887528668095], [16, 0.1762, 0.2625, 1.34, 0.108, 4102.4192317231, 495.686617327212], [27, 0.115, 0.3575, 1.04, 0.148, 4179.01703181483, 538.199920715909], [17, 0.183, 0.4825, 0.755, 0.188, 4234.17955561645, 533.944240829824], [14, 0.0504, 0.42, 0.845, 0.216, 4331.988912943, 122.456118069505], [25, 0.0487, 0.4825, 1.115, 0.176, 4335.39348658326, 208.040288454256], [20, 0.064, 0.3825, 1.505, 0.128, 4357.16741358047, 221.952417999471], [23, 0.0351, 0.305, 1.595, 0.132, 4474.59915078677, 140.829484473683], [21, 0.0402, 0.4625, 1.49, 0.148, 4516.31686357826, 144.900165430134], [18, 0.1915, 0.3625, 1.25, 0.148, 4582.62642527327, 599.759979891593], [21, 0.1609, 0.4475, 1.355, 0.14, 4629.99794640008, 581.094815524196], [28, 0.0657, 0.31, 1.325, 0.172, 4736.91998436384, 320.197182666606], [7, 0.0861, 0.48, 0.965, 0.236, 4770.95321102683, 105.757031172902], [23, 0.0538, 0.2525, 0.53, 0.284, 4801.0165073801, 216.747679182892], [15, 0.0453, 0.4375, 0.905, 0.248, 4842.31539056459, 117.53811609437], [25, 0.1626, 0.495, 1.4, 0.144, 4868.68432419737, 693.388053049229], [28, 0.1932, 0.465, 1.295, 0.14, 4878.23113060288, 925.655147897409], [17, 0.0657, 0.3475, 1.07, 0.24, 5085.33315087274, 194.779115795748], [9, 0.1184, 0.2725, 1.37, 0.204, 5098.17803430128, 189.526059447721], [27, 0.0742, 0.4325, 1.01, 0.248, 5240.60426030252, 344.345334843941], [18, 0.0793, 0.4725, 0.965, 0.264, 5276.72160577494, 245.131429306429], [12, 0.1099, 0.4075, 1.88, 0.152, 5289.08150204639, 229.889355917172], [21, 0.1643, 0.3375, 0.62, 0.296, 5488.82989072867, 600.697388940204], [19, 0.0827, 0.495, 0.74, 0.308, 5507.68919574696, 268.815393104766], [24, 0.1507, 0.3, 1.61, 0.176, 5565.29953827633, 630.936781094527], [16, 0.1388, 0.4225, 1.775, 0.176, 5586.01573132691, 384.468723429128], [13, 0.1524, 0.2875, 0.53, 0.336, 5613.79669163171, 348.897258897559], [21, 0.1014, 0.45, 2.0, 0.156, 5672.66540754316, 366.091472036387], [24, 0.1745, 0.4675, 0.92, 0.264, 5677.61203832696, 717.364680069612], [25, 0.0946, 0.34, 1.79, 0.18, 5689.697233026, 410.920852468344], [15, 0.064, 0.37, 1.82, 0.196, 5702.78095456785, 167.350312978048], [12, 0.1422, 0.365, 1.985, 0.172, 5790.17596199468, 298.843681748685], [29, 0.1694, 0.39, 1.22, 0.228, 5827.13158575004, 848.095322819911], [15, 0.1847, 0.3725, 1.745, 0.192, 5845.13142209241, 482.835557950085], [23, 0.1133, 0.275, 0.515, 0.348, 5851.53504310065, 455.699816890458], [16, 0.1167, 0.345, 0.695, 0.336, 5857.36034295038, 325.992532395089], [25, 0.1337, 0.3925, 1.205, 0.256, 5896.81172371643, 577.608339272753], [25, 0.1218, 0.35, 1.64, 0.208, 5929.06285498084, 528.555918941463], [21, 0.1286, 0.41, 1.145, 0.276, 5947.70348962619, 466.577076305226], [14, 0.1439, 0.295, 1.76, 0.208, 5949.12081183484, 354.082772839128], [21, 0.149, 0.325, 0.725, 0.324, 5969.5276717565, 545.395977171094], [17, 0.0742, 0.4375, 1.955, 0.196, 5974.23130032233, 217.782472933325], [24, 0.1609, 0.29, 1.475, 0.24, 6224.2523611557, 674.200451000885], [10, 0.115, 0.485, 0.89, 0.368, 6464.9901999873, 199.335290912619], [21, 0.1235, 0.3475, 1.4, 0.288, 6503.71036467834, 451.091783603391], [26, 0.1099, 0.4775, 1.745, 0.244, 6528.55942582999, 488.421754623039], [7, 0.1728, 0.3575, 1.595, 0.284, 6532.53027917004, 215.376883803421], [24, 0.1711, 0.26, 0.815, 0.344, 6551.9716001594, 718.603487814273], [21, 0.1932, 0.3625, 0.995, 0.332, 6673.09981981054, 704.613460384229], [22, 0.1167, 0.2925, 1.13, 0.34, 6739.28319240127, 448.547037066755], [25, 0.1881, 0.445, 1.1, 0.316, 6739.76889770949, 807.552864086552], [24, 0.1473, 0.2925, 1.91, 0.236, 6822.66569559159, 617.087592442291], [27, 0.1541, 0.45, 1.55, 0.276, 6855.34935501775, 713.567467646429], [7, 0.1677, 0.3725, 0.98, 0.396, 6986.31585128703, 208.696495072281], [16, 0.1048, 0.3125, 1.34, 0.352, 7083.33976424823, 293.634654013579], [17, 0.1813, 0.36, 1.67, 0.304, 7240.63701442228, 536.823713320632], [26, 0.0436, 0.35, 1.25, 0.384, 7256.77418191779, 196.7024281729], [13, 0.047, 0.4425, 1.445, 0.372, 7328.96526619711, 105.88816813403], [19, 0.1456, 0.425, 1.475, 0.344, 7383.12553893836, 477.655456989881], [10, 0.0725, 0.405, 1.055, 0.424, 7391.5243564537, 126.953190633291], [11, 0.1014, 0.3225, 1.22, 0.4, 7416.23754685341, 196.524062788126], [13, 0.1422, 0.41, 1.88, 0.308, 7417.9715624022, 321.63589249305], [8, 0.1983, 0.3325, 0.695, 0.46, 7431.184067965, 281.762922366412], [12, 0.1201, 0.45, 1.745, 0.332, 7438.66051251464, 249.923528289288], [28, 0.0385, 0.2725, 0.95, 0.436, 7438.74498638434, 188.211796541105], [28, 0.1915, 0.3625, 1.385, 0.328, 7480.62066681937, 928.658631842801], [25, 0.183, 0.41, 0.56, 0.444, 7512.85937843745, 789.01679754615], [17, 0.1082, 0.31, 1.985, 0.304, 7537.57830872898, 321.895533499017], [8, 0.0776, 0.26, 0.98, 0.448, 7566.97723674845, 110.929635656498], [20, 0.1558, 0.35, 1.01, 0.424, 7734.98120133336, 542.139829868121], [10, 0.0912, 0.4525, 1.055, 0.452, 7788.79007942348, 158.773435270832], [16, 0.0691, 0.3725, 1.085, 0.448, 7819.44069206793, 192.487452571429], [9, 0.0776, 0.3225, 1.145, 0.448, 7851.51783961429, 123.697562062554], [29, 0.0385, 0.325, 0.83, 0.488, 7929.12265986849, 194.020544454882], [21, 0.1439, 0.4125, 0.755, 0.488, 8134.54247427589, 521.934674326878], [16, 0.0317, 0.3125, 1.775, 0.396, 8159.83545889837, 88.8188791243363], [10, 0.1303, 0.4475, 1.22, 0.456, 8177.96630022894, 226.990059449204], [13, 0.1269, 0.43, 1.475, 0.42, 8185.47030067242, 286.343434423469], [11, 0.0776, 0.38, 1.73, 0.408, 8303.44746523305, 149.540587257553], [9, 0.1847, 0.415, 0.905, 0.5, 8308.3224310087, 291.587563286911], [18, 0.1371, 0.355, 1.91, 0.372, 8409.25255157423, 429.711336095461], [30, 0.1915, 0.405, 1.31, 0.404, 8412.09255166308, 989.859956741408], [15, 0.1201, 0.4, 1.97, 0.376, 8442.13380980279, 313.015761767285], [12, 0.1898, 0.43, 1.07, 0.484, 8470.45077847814, 395.960452836224], [23, 0.1473, 0.3325, 0.875, 0.496, 8502.9806739218, 589.543830820187], [22, 0.1966, 0.425, 1.565, 0.412, 8684.4956185472, 745.547711340491], [8, 0.0691, 0.44, 1.835, 0.46, 9101.06809466779, 97.0330246798941], [17, 0.1796, 0.27, 1.58, 0.464, 9182.35157608717, 536.056647351486], [18, 0.1575, 0.4475, 1.88, 0.488, 9933.20357443239, 488.468324495119], [12, 0.1915, 0.3625, 1.895, 0.5, 10028.4990294157, 402.554856363803], [10, 0.1728, 0.285, 0.605, 0.112, 10000000000L, 306.148882003536]]
sort=5
var=5
variables=6 # h w t nlam nstiff mass
gen=25 #generations of offspring 
gmat = [[0 for x in range(variables)] for y in range(sort*var)]
gmats=[[0 for x in range(variables)] for y in range(sort*var)]
best=[[0 for x in range(variables)] for y in range(gen)]

GenMat='gmat.txt'
dataFile=open(GenMat,'w')
dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass\n')
dataFile.close()
dataFile=open('gmatus.txt','w')
dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass\n')
dataFile.close()
dataFile=open('Bestruns.txt','w')
dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass\n')
dataFile.close()
Children='children.txt'
dataFile=open(Children,'w')
dataFile.write('All the Children \n')
dataFile.close()
mm=10000000000.0
for k in range(gen):
    ##Crossover and mutation
	gi=0
	sortedcount=0
	for i in range(sort):
		jj=0
		for j in range(var):
			if i==j:
				gmat[gi] = mat[i] ##error if mat is bad 
			else:
				mut1=0.0
				chance=random.uniform(0,1)
				if chance <0.14: #large mutation
					mut1=0.25
				elif chance <0.34: #medium mutation
					mut1=0.15
				else:
					mut1=0.05	#small mutation
				 
				# dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass,massribs\n')
				mutation1=mat[i][1]*mut1 #ribthickness
				mutation2=mat[i][2]*mut1#circleDiameter
				mutation3=mat[i][3]*mut1#sparThickness 
				mutation4=mat[i][4]*mut1#skinThickness

				v1=mat[i][1]-mutation1#ribthickness
				v2=mat[i][2]-mutation2 #circleDiameter
				v3=mat[i][3]-mutation3 #sparThickness 
				v4=mat[i][4]-mutation4 #skinThickness
				v5=mat[i][0]#nribs

				w1=mat[j][1]-mutation1
				w2=mat[j][2]-mutation2
				w3=mat[j][3]-mutation3
				w4=mat[j][4]-mutation4
				w5=mat[j][0]#nribs 

				fg=random.randint(0, 1)
				if fg==0:
					gmat[gi][4]=w4#skin thickness
				else:
					gmat[gi][4]=v4
				fg=random.randint(0, 1)
				if fg==0:
					gmat[gi][1]=v1
				else:
					gmat[gi][1]=v1
				fg=random.randint(0, 1)
				if fg==0:
					gmat[gi][2]=w2
				else:
					gmat[gi][2]=v2
				fg=random.randint(0, 1)
				if fg==0:
					gmat[gi][3]=w3
				else:
					gmat[gi][3]=v3
				fg=random.randint(0, 1)
				if fg==0:
					gmat[gi][0]=w5
				else:
					gmat[gi][0]=v5
				#temp=[numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass,massribs]
				numberOfRibs=int(gmat[gi][0])
				ribThickness=gmat[gi][1]
				circleDiameter=gmat[gi][2]
				sparThickness=gmat[gi][3]
				skinThickness=gmat[gi][4]

				# MassProperties=mdb.models['Model-1'].rootAssembly.getMassProperties()
				# mass=MassProperties['mass']
				gmat[gi][5]=1.0 ##placeholder 

				# numberOfRibs = int(round(n_ribs[i]))
				# skinThickness = t_skin[i]            # in
				# sparThickness = t_spars[i]             # in
				# ribThickness = t_ribs[i]               # in
				# circleDiameter = r_cuts[i]				#percentage 
				seedSize = 2
				skinMaterialComposite = generateCompositeDictionary('Composite', .10, 5, skinThickness, [20600000.0, 1493889.0, 0.27, 1044000.0, 1044000.0, 1044000.0])
				# loadCase = 'takeoff'
				loadFactor = 1.5
				airfoilCoordinates = 'C:\\temp\\proj88\\CoordinateData\\normalizedCoordinates.csv'
				aspectRatio = 6
				taperRatio = 0.25
				wingArea = feetToInches(1570.0*12.0)       # square in
				leadingEdgeSweep = radians(32)         # rad
				wingSpan = feetToInches(97.1)          # in
				halfWingSpan = wingSpan/2.0            # in
				rootChord = chord(0)                   # in
				tipChord = chord(halfWingSpan)         # in

				# Spar information ## TODO: This should be a dictionary that can be passed similar to the stringers
				frontSparLocation = 0.155
				backSparLocation = 0.695
				middleSparLocation = (1.0/2.0)*(frontSparLocation + backSparLocation)
				sparLocations = [0.155, 0.55, 0.795]
				maxLoad = (0.5)*(planeWeight/wingArea) # psi
				xValue = halfWingSpan - (0.25)*halfWingSpan/(float(numberOfRibs) - 1) # Get the x value a quarter step back from the tip rib
				pressureLoad = -maxLoad*sqrt( 1.0 - pow((halfWingSpan - (0.25)*halfWingSpan/(float(numberOfRibs) - 1))/halfWingSpan, 2) )
				ellipticalLoad = '-.30669674451*sqrt ( 1-pow(Z/570,2))'
				locationOfSecondToLastRib = (wingSpan/2) - (wingSpan/2)/(numberOfRibs - 1)
				
				Mdb()
				
				# This allows ABAQUS to use findAt() instead of mask
				session.journalOptions.setValues(replayGeometry = COORDINATE, recoverGeometry = COORDINATE)

				locationOfSecondToLastRib = (wingSpan/2) - (wingSpan/2)/(numberOfRibs - 1)

				# Create the rib parts and assign materials, sections, and create instances
				generateRibs(numberOfRibs, ribMaterial, ribThickness, sparLocations, circleDiameter)

				# Create the skin
				generateSkin(numberOfRibs, skinMaterialComposite, skinThickness)

				# Partition the ribs (delete the code in generateRibs())
				#### TODO: AllRibs needs to use the newly partitioned ribs
				partitionRibs(numberOfRibs)

				# Partition the skin where the spars are going to be and generate the spars by lofting
				generateSpars(sparLocations, sparMaterial, sparThickness)

				# Create the stringers on the skin
				generateStringers('Stringers', stringerDictionary)

				# Merge all the parts together so they form one rigid body
				mergeWing()

				# Rotate the wing
				#if loadCase == 'cruise':
				#    rotateWing(0.33, 32, 60)

				# Generate the meshes
				generateMesh(seedSize)

				# Create a step
				mdb.models['Model-1'].StaticStep(name='Step-1', previous='Initial')

				# Create loads and boundary conditions
				generateLoadsAndBCs(ellipticalLoad, loadFactor, pressureLoad, numberOfRibs) #numberOfRibs

				# Create and run a job
				createAndRunJob()
				
				ModelName='Model-1'
				MassProperties=mdb.models['Model-1'].rootAssembly.getMassProperties()
				
				mass=MassProperties['mass']
				gmat[gi][5]=mass
				MassProperties2=mdb.models['Model-1'].parts['AllRibs'].getMassProperties()
				massribs=MassProperties2['mass']
				# stress=getResults(ModelName)
				# deflection=getResults(ModelName)
				d,f=getResults(ModelName)
				stress=d
				deflection=f	
				dataFile=open('children.txt','a')
				dataFile.write('%6f,%6f\n'%(mass,deflection))
				dataFile.close()
				#temp=[numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,mass,massribs]
				if stress<73000.0 or deflection<60.0:
					#print "run"
					GenMat='gmatus.txt'
					dataFile=open(GenMat,'a')
					dataFile.write ('%6f,%6f,%6f,%6f,%6f,%6f'% (numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,gmat[gi][5]))
					dataFile.write('\n')
					gmats[sortedcount][0]=numberOfRibs
					gmats[sortedcount][1]=ribThickness
					gmats[sortedcount][2]=circleDiameter
					gmats[sortedcount][3]=sparThickness
					gmats[sortedcount][4]=skinThickness
					gmats[sortedcount][5]=mass
					sortedcount=sortedcount+1
			dataFile=open('gmat.txt','a')
			dataFile.write ('%6f,%6f,%6f,%6f,%6f,%6f'% (gmat[gi][0],gmat[gi][1],gmat[gi][2],gmat[gi][3],gmat[gi][4],gmat[gi][5]))
			dataFile.write('\n')
			
			gi=gi+1
	dataFile.close()


	gmt=sorted(gmats, key=lambda row:row[5])
	GenMat='gmats.txt'
	dataFile=open(GenMat,'w')
	dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,massT\n')
	count = 0
	for i in gmt:
		for item in gmt[count]:
			dataFile.write("%s," % item)
		count = count + 1
		dataFile.write('\n')
	dataFile.close()
	dataFile=open('Bestruns.txt','a')
	best[k][0]=gmats[0][0]
	best[k][1]=gmats[0][1]
	best[k][2]=gmats[0][2]
	best[k][3]=gmats[0][3]
	best[k][4]=gmats[0][4]
	best[k][5]=gmats[0][5]

	dataFile.write('%6f,%6f,%6f,%6f,%6f,%6f\n' % (gmats[0][0],gmats[0][1],gmats[0][2],gmats[0][3],gmats[0][4],gmats[0][5]))
	dataFile.close()
	kk=k+1
	print('Gen: %3d'%kk)
   
Bestruns=sorted(best, key=lambda row:row[5])
dataFile=open('Bestrunss.txt','w')
dataFile.write('numberOfRibs,ribThickness,circleDiameter,sparThickness,skinThickness,massT\n')
count = 0
for i in Bestruns:
	for cell in Bestruns[count]:
		dataFile.write("%s," % cell)
	count = count + 1
	dataFile.write('\n')
dataFile.close()
count=0
for i in Bestruns:
	if Bestruns[count][5]==0:
		Bestruns[count][5]==10E10
print 'DONE'
