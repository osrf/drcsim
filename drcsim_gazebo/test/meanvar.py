#!/usr/bin/env python

import math
import optparse
import sys

def read_data(fname, col, sep):
    data = []
    with open(fname, 'r') as f:
        for l in f.readlines():
            l = l.strip()
            if len(l) == 0 or l[0] == '%' or l[0] == '#':
                continue
            cols = l.split(sep)
            data.append(float(cols[col]))
    return data

# Modified from:
# http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
def compute_stats(data, hist):
    n = 0.0
    mean = 0.0
    M2 = 0.0
    minimum = sys.float_info.max
    maximum = sys.float_info.min
    H = {}
 
    for x in data:
        if x < minimum:
            minimum = x
        if x > maximum:
            maximum = x
        if hist:
            # Assume that we're looking at discrete millisecond timesteps
            intx = int(round(x*1e3))
            if intx not in H:
                H[intx] = 0
            H[intx] += 1
        n = n + 1
        delta = x - mean
        mean = mean + delta/n
        M2 = M2 + delta*(x - mean)
 
    variance = M2/(n - 1)
    stddev = math.sqrt(variance)
    return (mean, stddev, minimum, maximum, H)

def parse_args():
    parser = optparse.OptionParser()
    parser.add_option("-c", "--column", dest="column",
                      help="which column index to use", default=0)
    parser.add_option("-s", "--separator", dest="sep",
                      help="which separator character to use", default=' ')
    parser.add_option("-g", "--histogram", action="store_true", dest="hist",
                      help="also compute and print histogram data", default=False)
    return parser.parse_args()

if __name__ == '__main__':
    options, args = parse_args()
    if len(args) != 1:
        print 'Must supply an input file name'
        sys.exit(1)

    data = read_data(args[0], int(options.column), options.sep)
    (mean, stddev, minimum, maximum, H) = compute_stats(data, options.hist)

    print '# num samples: %d'%(len(data))
    print '# mean: %f'%(mean)
    print '# stddev: %f'%(stddev)
    print '# minimum: %f'%(minimum)
    print '# maximum: %f'%(maximum)

    if options.hist:
        print '# Histogram data'
        print '# value count pct'
        for k in sorted(H.iterkeys()):
            print '%d %f %f'%(k, H[k], H[k]/float(len(data)))
