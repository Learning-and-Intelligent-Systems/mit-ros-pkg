#!/usr/bin/env python
from numpy import *
import matplotlib.pyplot as plt
from itertools import count, izip
from pylab import *
if __name__ == '__main__':
    """
    0.049    35617    0.052    29010
0.017592612085759    16461.7228837081    0.014601369798755    9031.11160378389
0.04    35142    0.051    32373
0.021011901389451    7626.45129139366    0.027847800631289    8780.60010477644
0.061    40331    0.06    41283.5
0.014851647335277    7690.41424669142    0.013392784126785    5969.55055817996
0.06    52208    0.0555    57301.5
0.016117278513033    8581.65128049375    0.025184675045409    26912.3879216033
0.064    56984.5    0.047    51175
0.02413020237517    9791.2397104078    0.025272514714606    33800.0446331066
0.089    67695    0.053    50994
0.007661592523751    29721.417928827    0.071577231016574    71688.0516041551
0.097    49403    0.083    94004
0.020537770083434    5841.92569278316    0.02125581153025    53911.4989651249
    """
    medians_100k = [0.049,0.04,0.061,0.06,0.064,0.089,0.097]
    median_stdevs_100k = [0.017592612085759,0.021011901389451,0.014851647335277,0.016117278513033,
                          0.02413020237517,0.007661592523751,0.020537770083434]
    median_ks_100k = [35617,35142,40331,52208,56984.5,67695,49403]
    median_ks_stdevs_100k = [16461.7228837081,7626.45129139366,7690.41424669142,8581.65128049375,
                            9791.2397104078,29721.417928827,5841.92569278316]
    medians_200k = [0.052,0.051,0.06,0.0555,0.047,0.053,0.083]
    median_stdevs_200k = [0.014601369798755,0.027847800631289, 0.013392784126785,0.025184675045409,
                           0.025272514714606,0.071577231016574,0.02125581153025]
    median_ks_200k = [29010,32373,41283.5,57301.5,51175,50994,94004]
    median_ks_stdevs_200k = [9031.11160378389,8780.60010477644,5969.55055817996,26912.3879216033,
                             33800.0446331066,71688.0516041551,53911.4989651249]
    
    fig=figure()
    ylabel('e_min median')
    xlabel('class')
    xlim([0,8])
    b1=errorbar([1,2,3,4,5,6,7], medians_100k, yerr=median_stdevs_100k, fmt='ro',label = 'kmax=100k')
    b2=errorbar([1,2,3,4,5,6,7], medians_200k, yerr=median_stdevs_200k, fmt='go', label ='kmax=200k')
    legend(loc='upper left')
    fig.savefig('e_min-plot.pdf')
    show()
    raw_input()
    fig = figure()
    ylabel('k median')
    xlabel('class')
    xlim([0,8])
    b1=errorbar([1,2,3,4,5,6,7], median_ks_100k, yerr=median_ks_stdevs_100k, fmt='ro',label = 'kmax=100k')
    b2=errorbar([1,2,3,4,5,6,7], median_ks_200k, yerr=median_ks_stdevs_200k, fmt='go', label ='kmax=200k')
    legend(loc='upper left')
    fig.savefig('k-plot.pdf')
    show()
    raw_input()
    