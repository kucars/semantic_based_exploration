import matplotlib.pyplot as plt
import csv
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

volumetricCoverage_PE = []
environmentGain_PE = []
comulativeGain_PE= [] 
freeCellsCounter_PE = [] 
occupiedCellsCounter_PE = [] 
uknownCellsCounter_PE = [] 

theoriticalNumberOfCells_PE = [] 
sumTypesOfCells_PE =[] 
sumOccuAndFreeCells_PE = []
inLoopCounter_PE = [] 



j = []
i = 0
iterationNumber = 30 

'''
The following code reads data from csv file for results from pure entropy 
'''
with open('metric2_PE.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
 	if(i<iterationNumber):
	        volumetricCoverage_PE.append(row[0])
        	environmentGain_PE.append(row[1])
	        comulativeGain_PE.append(row[2])
	        freeCellsCounter_PE.append(row[3])
	        occupiedCellsCounter_PE.append(row[4])
	        uknownCellsCounter_PE.append(row[5])
	        theoriticalNumberOfCells_PE.append(row[6])
	        sumTypesOfCells_PE.append(row[7])
	        sumOccuAndFreeCells_PE.append(row[8])
		inLoopCounter_PE.append(row[9])
		i = i + 1
		j.append(i)


volumetricCoverage_PD = []
environmentGain_PD = []
comulativeGain_PD= [] 
freeCellsCounter_PD = [] 
occupiedCellsCounter_PD = [] 
uknownCellsCounter_PD = [] 

theoriticalNumberOfCells_PD = [] 
sumTypesOfCells_PD =[] 
sumOccuAndFreeCells_PD = []
inLoopCounter_PD = [] 

l = 0 
with open('metric2_PD.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
 	if(l<iterationNumber):
	        volumetricCoverage_PD.append(row[0])
        	environmentGain_PD.append(row[1])
	        comulativeGain_PD.append(row[2])
	        freeCellsCounter_PD.append(row[3])
	        occupiedCellsCounter_PD.append(row[4])
	        uknownCellsCounter_PD.append(row[5])
	        theoriticalNumberOfCells_PD.append(row[6])
	        sumTypesOfCells_PD.append(row[7])
	        sumOccuAndFreeCells_PD.append(row[8])
		inLoopCounter_PD.append(row[9])
		l = l + 1
		


volumetricCoverage_AE = []
environmentGain_AE = []
comulativeGain_AE= [] 
freeCellsCounter_AE = [] 
occupiedCellsCounter_AE = [] 
uknownCellsCounter_AE = [] 

theoriticalNumberOfCells_AE = [] 
sumTypesOfCells_AE =[] 
sumOccuAndFreeCells_AE = []
inLoopCounter_AE = [] 

m = 0 
with open('metric2_AE.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
 	if(m<iterationNumber):
	        volumetricCoverage_AE.append(row[0])
		print row[0]
        	environmentGain_AE.append(row[1])
	        comulativeGain_AE.append(row[2])
	        freeCellsCounter_AE.append(row[3])
	        occupiedCellsCounter_AE.append(row[4])
	        uknownCellsCounter_AE.append(row[5])
	        theoriticalNumberOfCells_AE.append(row[6])
	        sumTypesOfCells_AE.append(row[7])
	        sumOccuAndFreeCells_AE.append(row[8])
		inLoopCounter_AE.append(row[9])
		m = m + 1
		


print volumetricCoverage_AE


plt.figure(1)
plt.plot(j,volumetricCoverage_PE,'r',j,volumetricCoverage_PD,'b',j,volumetricCoverage_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Percentage of coverage')
plt.title('Volumetric Coverage')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.grid(True)
plt.savefig('Volumetric Coverage.png')
plt.show()

'''
plt.figure(2)
plt.plot(j,environmentGain_PE,'r',j,environmentGain_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('Environment Information gain')
plt.title('Environment IG')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('EnvironmentIG.png')
plt.show()
'''

plt.figure(2)
plt.subplot(311)
plt.plot(j,environmentGain_PE,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Environment Information gain')
plt.title('Environment IG')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
plt.legend(handles=[red_patch])
plt.grid(True)


plt.subplot(312)
plt.plot(j,environmentGain_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('Environment Information gain')
plt.title('Environment IG')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[blue_patch])
plt.grid(True)

plt.subplot(313)
plt.plot(j,environmentGain_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Environment Information gain')
plt.title('Environment IG')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[black_patch])
plt.grid(True)

plt.tight_layout()
plt.savefig('EnvironmentIG.png')
plt.show()



'''
plt.figure(3)
plt.plot(j,comulativeGain_PE,'r',j,comulativeGain_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('Commulative IG')
plt.title('Commulative IG')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('CumulativeIG.png')
plt.show()
'''


plt.figure(3)
plt.subplot(311)
plt.plot(j,comulativeGain_PE,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Cumulative IG')
plt.title('Cumulative IG')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
plt.legend(handles=[red_patch])
plt.grid(True)

plt.subplot(312)
plt.plot(j,comulativeGain_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('Cumulative IG')
plt.title('Commulative IG')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[blue_patch])
plt.grid(True)


plt.subplot(313)
plt.plot(j,environmentGain_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Environment Information gain')
plt.title('Environment IG')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[black_patch])
plt.grid(True)

plt.tight_layout()
plt.savefig('CumulativeIG.png')
plt.show()



plt.figure(4)
plt.plot(j,freeCellsCounter_PE,'r',j,freeCellsCounter_PD,'b',j,freeCellsCounter_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Number of free volumetric cells')
plt.title('Free Cells Counter')
plt.legend()
plt.grid(True)
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.savefig('freeCellsCounter.png')
plt.show()


plt.figure(5)
plt.plot(j,occupiedCellsCounter_PE,'r',j,occupiedCellsCounter_PD,'b',j,freeCellsCounter_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Number of Occupied volumetric cells')
plt.title('Occupied Cells Coubter')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.grid(True)
plt.savefig('occupiedCellsCounter.png')
plt.show()


plt.figure(6)
plt.plot(j,uknownCellsCounter_PE,'r',j,uknownCellsCounter_PD,'b',j,uknownCellsCounter_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Unknow Cells Counter')
plt.title('UknownCellsCounter')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.grid(True)
plt.savefig('UknownCellsCounterG.png')
plt.show()

'''
plt.figure(7)
plt.plot(j,theoriticalNumberOfCells_PE,'r',j,theoriticalNumberOfCells_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('Number Of Voxels')
plt.title('Number Voxels In Thoery')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('TheoryNumberOfVoxels.png')
plt.show()
'''
'''
plt.figure(8)
plt.plot(j,sumTypesOfCells_PE,'r',j,sumTypesOfCells_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('free + occupied + unknown voxels')
plt.title('addingAllVoxels')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('addingAllVoxels.png')
plt.show()
'''

plt.figure(8)
plt.plot(j,sumOccuAndFreeCells_PE,'r',j,sumOccuAndFreeCells_PD,'b',j,sumOccuAndFreeCells_AE,'k')
plt.xlabel('Number of Iteration')
plt.ylabel('Free + Occupied')
plt.title('Known Cells')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
black_patch = mpatches.Patch(color='black', label='Average Entropy')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.grid(True)
plt.savefig('KnownCells.png')
plt.show()

'''
plt.figure(9)
plt.plot(j,inLoopCounter_PE,'r',j,inLoopCounter_PD,'b')
plt.xlabel('Number of Iteration')
plt.ylabel('counting numebr of iteration cells')
plt.title('counting number of iteration cells')
red_patch = mpatches.Patch(color='red', label='Pure Entropy')
blue_patch = mpatches.Patch(color='blue', label='Point Desity')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('countingNOfIterationCells.png')
plt.show()

'''

