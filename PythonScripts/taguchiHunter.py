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
from math import atan
from Post_P_Script import getResults
from mainTest import main

def linspace(list, i):
	return ( list[0] + (list[1] - list[0])*(i - 1)/3.0 )

def taguchiMain():
	# Define the Taguchi L16b
	#     1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16
	X1 = [1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4] # number or ribs 
	X2 = [1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4] # thickness of ribs
	X3 = [1, 2, 3, 4, 2, 1, 4, 3, 3, 4, 1, 2, 4, 3, 2, 1] # hole size
	X4 = [1, 2, 3, 4, 3, 4, 1, 2, 4, 3, 2, 1, 2, 1, 4, 3] # thickness of spars 
	X5 = [1, 2, 3, 4, 4, 3, 2, 1, 2, 1, 4, 3, 3, 4, 1, 2] # thickness of skins 

	# Define the bounds on the design variables
	numberOfRibBounds = [5, 30]
	ribThicknessBounds = [0.03, 0.2] 
	circleDiameterBounds = [0.25, 0.5] 
	sparThicknessBounds = [0.5, 2.0]
	skinThicknessBounds = [0.025, 0.5]

	# Generate the design space
	numberOfRibsList = [ int(round(linspace(numberOfRibBounds, element))) for element in X1]
	ribThicknessList = [ linspace(ribThicknessBounds, element) for element in X2 ]
	circleDiametersList = [ linspace(circleDiameterBounds, element) for element in X3 ]
	sparThicknessList = [ linspace(sparThicknessBounds, element) for element in X4 ]
	skinThicknessList = [ linspace(skinThicknessBounds, element) for element in X5 ]

	# Write to the data file
	DataFile = open('DOE_Data.txt','w')
	DataFile.write('n_ribs\t t_ribs\t r_cuts\t t_spars\t t_skin\t totalmass\t deflection\t stress\t allRibsMass\n')
	DataFile.close()

	for i in range(16):

		# Create a dictionary for the design variables
		designVariablesDictionary = {
			'numberOfRibs': numberOfRibsList[i],
			'skinThickness': skinThicknessList[i],           # inches
			'sparThickness': sparThicknessList[i],           # inches
			'ribThickness': ribThicknessList[i],             # inches
			'circleDiameter': circleDiametersList[i]	     # percentage of chord
		}
		
		loadFactor = 1.5
		seedSize = 4
		loadCondition = 'takeoff'
		results = main(designVariablesDictionary, loadCondition, loadFactor, seedSize)

		numberOfRibs = results['numberOfRibs']
		skinThickness = results['skinThickness']
		sparThickness = results['sparThickness']
		ribThickness = results['ribThickness']
		circleDiameter = results['circleDiameter']
		mass = results['totalMass']
		deflection = results['deflection']
		stress = results['stress']
		massribs = results['massRibs']

		DataFile = open('DOE_Data.txt','a')
		DataFile.write('\n%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n' % (numberOfRibs, ribThickness, circleDiameter, sparThickness, skinThickness, mass, deflection, stress, massribs))
		DataFile.close()

taguchiMain()