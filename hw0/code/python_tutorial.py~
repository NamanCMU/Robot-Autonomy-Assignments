#!/usr/bin/env python
import numpy as np

def print_list(l):
    print l

def sort_manual(shops):

    shops_sorted = []

    # TODO: Here implement manual sorting using loops
    for key,value in shops.iteritems():
	temp_set = [key,str(value)]
	shops_sorted.append(temp_set)
	
    for i in xrange(0,len(shops_sorted)):
	for j in xrange(0,len(shops_sorted) - 1):
		if shops_sorted[j][1] < shops_sorted[j + 1][1]:
			temp = shops_sorted[j + 1]
			shops_sorted[j + 1] = shops_sorted[j]
			shops_sorted[j] = temp
	
    print 'Manual sorting result: '
    print_list(shops_sorted)

def sort_python(shops):
    
    shops_sorted = []

    #TODO: Here implement sorting using pythons build in sorting functions
    shops_sorted = sorted(shops.items(), key=lambda x: x[1],reverse = True)
    print 'Python sorting result: '
    print_list(shops_sorted)

def sort_numpy(shops):
    
    shops_sorted = []
    # TODO: Here implement sorting using numpys build-in sorting function
    length = max(map(len, shops))
    stringtype = '|S' + str(length)
    shops_sorted = np.array([(key,value) for key,value in shops.iteritems()], \
		   dtype=[('a',stringtype),('b',stringtype)])
		   
    shops_sorted = list(reversed(np.sort(shops_sorted,order=['b'],axis=0).tolist()))    
    print 'Numpy sorting result: '
    print_list(shops_sorted)

def main():

    shops = {}
    shops['21st Street'] = 0.9
    shops['Voluto'] = 0.6
    shops['Coffee Tree'] = 0.45
    shops['Tazza D\' Oro'] = 0.75
    shops['Espresso a Mano'] = 0.95
    shops['Crazy Mocha'] = 0.35
    shops['Commonplace'] = 0.5

    sort_manual(shops)
    sort_python(shops)
    sort_numpy(shops)
    

if __name__ == "__main__":
    main()
