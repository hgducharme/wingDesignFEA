import matplotlib.pyplot as plt
import numpy as np

def retrieveData(filePath):
    mass = []
    deflection = []

    with open(filePath,"r") as file:
        for line in file:
            parts = line.split(',')
            mass.append(float(parts[0]))
            deflection.append(float(parts[1]))

    return mass, deflection

# Define the case you want to do
case = 'subsonic'

# Extract the data from the text file
filePath = 'C:/Users/Hunter Ducharme/Desktop/' + case + 'Children.txt'
massTemp, deflectionTemp = retrieveData(filePath)

# Get the biggest mass and deflection so we can normalize when we plot
biggestMass = max(massTemp)
biggestDeflection = max(deflectionTemp)

# Turn the lists into groups based on the number of children in each generations
numberChildrenPerGeneration = 25
mass = [massTemp[i:i+25] for i in range(0, len(massTemp), numberChildrenPerGeneration)]
deflection = [deflectionTemp[i:i+25] for i in range(0, len(deflectionTemp), numberChildrenPerGeneration)]
weightMass = 0.5
weightDeflection = 0.5

for i in range(len(mass)):

        # Normalize the data using min-max normalization
        normalizedMass = [ childMass/biggestMass for childMass in mass[i] ]
        normalizedDeflection = [ childDeflection/biggestDeflection for childDeflection in deflection[i] ]
        normalizedMassRange = np.linspace(min(normalizedMass), max(normalizedMass), 16).tolist()

        # Calculate the objective function for each child in the population and find the value and index of the smallest value
        objective = [ (weightMass*normalizedMass[i]) + (weightDeflection*normalizedDeflection[i]) for i in range(len(normalizedMass)) ]
        smallestObjective = min(objective)
        indexOfBestDesign = objective.index(smallestObjective)

        # Get the design that corresponds to the smallest objective function
        bestMass = normalizedMass[indexOfBestDesign]
        bestDeflection = normalizedDeflection[indexOfBestDesign]

        # Calculate the line that goes through the best design
        line = [ (-(weightDeflection/weightMass)*(normalizedMassRange[i] - bestMass) + bestDeflection) for i in range(len(mass)) ]

        # Plot all the data and save it to a .png file
        #plt.style.use('classic')
        figure = plt.figure()
        plt.scatter(normalizedMass, normalizedDeflection, label = "Child")
        lineLabel = "Line of best design, fHat = " + str(round(smallestObjective, 2))
        plt.plot(normalizedMassRange, line, c = "red", linestyle=':', label = lineLabel)
        plotTitle = case.title() + ' Population ' + str(i + 1)
        figure.suptitle(plotTitle)
        plt.xlabel('Mass')
        plt.ylabel('Deflection')
        plt.axis([0, 1, 0, 1])
        plt.legend(loc = "lower left")
        figureName = 'C:/Users/Hunter Ducharme/Desktop/' + case + 'Pictures/population' + str(i+1) + '.png'
        plt.savefig(figureName, bbox_inches='tight')
        plt.close(figure)