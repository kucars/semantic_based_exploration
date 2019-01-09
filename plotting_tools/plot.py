import matplotlib.pyplot as plt
import csv
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

volumetric_coverage =[]
Information_gain_entropy = []
semantic_gain = []
semantic_Information_gains =[]
num_of_free_cells = []
num_of_occupied_cells = []
num_of_unknown_cells = []
num_of_known_cells = []
num_of_all_cells = []
traveled_distance =[] 

j = []
i = 0
iterationNumber = 500

'''
The following code reads data from csv file for results from pure entropy
'''
with open('gains.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
     if(i<iterationNumber):
		volumetric_coverage.append(row[0])
		Information_gain_entropy.append(row[1])
		semantic_gain.append(row[2])
		semantic_Information_gains.append(row[3])
		num_of_free_cells.append(row[4])
		num_of_occupied_cells.append(row[5])
		num_of_unknown_cells.append(row[6])
		num_of_known_cells.append(row[7])
		num_of_all_cells.append(row[8])
		traveled_distance.append(row[9])
		i = i + 1
		j.append(i)

print len(j)

plt.figure(1)
plt.plot(j,volumetric_coverage,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Percentage of Volumetric Coverage')
plt.title('Volumetric Coverage')
#red_patch = mpatches.Patch(color='red', label='Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Volumetric Coverage.png')
plt.show()


plt.figure(2)
plt.plot(j,Information_gain_entropy,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Information Gain')
plt.title('Information Gain')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Information gain entroy.png')
plt.show()

plt.figure(3)
plt.plot(j,semantic_gain,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Semantic Gain')
plt.title('Semantic Gain')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('semantic gain entropy.png')
plt.show()


plt.figure(4)
plt.plot(j,semantic_Information_gains,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Total Gain')
plt.title('Total Gain')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Total gain entropy.png')
plt.show()


plt.figure(5)
plt.plot(j, num_of_free_cells,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Number of Free Cells')
plt.title('Number of Free Cells vs iteration')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Free Cells.png')
plt.show()

plt.figure(6)
plt.plot(j, num_of_occupied_cells,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Number of occupied cells')
plt.title('Number of occupied cells vs iteration')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('OccupiedCells.png')
plt.show()

plt.figure(7)
plt.plot(j, num_of_unknown_cells,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('Number of Unknown Cells')
plt.title('Number of Unknown Cells vs iteration')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Unknown Cells.png')
plt.show()




plt.figure(8)
plt.plot(j, num_of_known_cells,'r' )
plt.xlabel('Number of Iteration')
plt.ylabel('Number of Known Cells')
plt.title('Number of Known Cells vs iteration')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('Known Cells.png')
plt.show()



plt.figure(9)
plt.plot(j, num_of_unknown_cells,'b',j, num_of_free_cells,'k',j, num_of_occupied_cells,'r' )
plt.xlabel('Number of Iteration')
plt.ylabel('Cells type ')
plt.title('Cells type vs Iteration')
red_patch = mpatches.Patch(color='red', label='occupied cells')
blue_patch = mpatches.Patch(color='blue', label='unknown cells ')
black_patch = mpatches.Patch(color='black', label='free cells ')
plt.legend(handles=[red_patch,blue_patch,black_patch])
plt.grid(True)
plt.savefig('ThreeCellsType.png')
plt.show()



plt.figure(10)
plt.plot(j, num_of_known_cells,'r',j, num_of_unknown_cells,'b' )
plt.xlabel('Number of Iteration')
plt.ylabel('Knonw and Unknown Cells')
plt.title('Number of Cells vs iteration')
red_patch = mpatches.Patch(color='red', label='known cells')
blue_patch = mpatches.Patch(color='blue', label='unknown cells ')
plt.legend(handles=[red_patch,blue_patch])
plt.grid(True)
plt.savefig('TwoCellsType.png')
plt.show()




plt.figure(11)
plt.plot(j, num_of_all_cells,'r')
plt.xlabel('Number of Iteration')
plt.ylabel('num_of_all_cells')
plt.title('num_of_all_cells vs iteration')
#red_patch = mpatches.Patch(color='red', label='Pure Entropy')
#plt.legend(handles=[red_patch])
plt.grid(True)
plt.savefig('num_of_all_cells.png')
plt.show()
















