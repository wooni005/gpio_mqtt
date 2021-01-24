#!/usr/bin/python3

# Reminder: best is to have a minimum of 3 halve periods (3x10ms) in a row for pulseOn and pulseOff
# This because when stearing 9ms on, some periods will not be switch on, because it fits perfectly between the 10ms zero crossings.
# Solution to solve this, would be zero-crossing detection and syncing the pulse on that.
# When using 10ms pulseOn, you will get more often 2 halve periods, twice as much power. Same story is for pulseOff.
dimming = [
    [0, 0],      # 0=0%
    [1, 79],     # 1=1,25%
    [1, 79],     # 2=1,25%
    [1, 39],     # 3=2,5%
    [3, 77],     # 4=3,75%
    [1, 19],     # 5=5%
    [5, 75],     # 6=6,25%
    [3, 37],     # 7=7,5%
    [3, 37],     # 8=7,5%
    [7, 73],     # 9=8,75%
    [1,  9],     # 10=10%
    [9, 71],     # 11=11,25%
    [2, 14],     # 12=12,5%
    [2, 14],     # 13=12,5%
    [11, 69],    # 14=13,75
    [3, 17],     # 15=15%
    [13, 67],    # 16=16,25%
    [7, 33],     # 17=17,5%
    [7, 33],     # 18=17,5%
    [15, 65],    # 19=18,75%
    [2, 8],      # 20=20%
    [17, 63],    # 21=21,25%
    [9, 31],     # 22=22,5%
    [9, 31],     # 23=22,5%
    [19, 61],    # 24=23,75%
    [2, 6],      # 25=25%
    [21, 59],    # 26=26,25%
    [11, 29],    # 27=27,5%
    [11, 29],    # 28=27,5%
    [23, 57],    # 29=28,75%
    [3, 7],      # 30=30%
    [25, 55],    # 31=31,25%
    [13, 27],    # 32=32,5%
    [13, 27],    # 33=32,5%
    [27, 53],    # 34=33,75%
    [7, 13],     # 35=35%
    [29, 51],    # 36=36,25%
    [3, 5],      # 37=37,5%
    [3, 5],      # 38=37,5%
    [31, 49],    # 39=38,75%
    [4, 6],      # 40=40%
    [33, 47],    # 41=41,25%
    [17, 23],    # 42=42,5%
    [17, 23],    # 43=42,5%
    [35, 45],    # 44=43,75%
    [9, 11],     # 45=45%
    [37, 43],    # 46=46,25%
    [19, 21],    # 47=47,5%
    [19, 21],    # 48=47,5%
    [39, 41],    # 49=48,75%
    [2, 2],      # 50=50%
    [41, 39],    # 51=51,25%
    [21, 19],    # 52=52,5%
    [21, 19],    # 53=52,5%
    [43, 37],    # 54=52,75%
    [11, 9],     # 55=55%
    [45, 35],    # 56=56,25%
    [23, 17],    # 57=57,5%
    [23, 17],    # 58=57,5%
    [47, 33],    # 59=58,75%
    [6, 4],      # 60=60%
    [49, 31],    # 61=61,25%
    [5, 3],      # 62=62,5%
    [5, 3],      # 63=62,5%
    [51, 29],    # 64=63,75%
    [13, 7],     # 65=65%
    [53, 27],    # 66=66,75%
    [27, 13],    # 67=67,5%
    [27, 13],    # 68=67,5%
    [55, 25],    # 69=68,75%
    [7, 3],      # 70=70%
    [57, 23],    # 71=71,25%
    [29, 11],    # 72=72,5%
    [29, 11],    # 73=72,5%
    [59, 21],    # 74=73,75%
    [3, 1],      # 75=75%
    [61, 19],    # 76=76,25%
    [31, 9],     # 77=77,5%
    [31, 9],     # 78=77,5%
    [63, 17],    # 79=78,75%
    [8, 2],      # 80=80%
    [65, 15],    # 81=81,25%
    [33, 7],     # 82=82,5%
    [33, 7],     # 83=82,5%
    [67, 13],    # 84=83,75%
    [17, 3],     # 85=85%
    [69, 11],    # 86=86,25%
    [7, 1],      # 87=87,5%
    [7, 1],      # 88=87,5%
    [71, 9],     # 89=88,75%
    [9, 1],      # 90=90%
    [73, 7],     # 91=91,25%
    [37, 3],     # 92=92,5%
    [37, 3],     # 93=92,5%
    [75, 5],     # 94=93,75%
    [19, 1],     # 95=95%
    [77, 3],     # 96=96,25%
    [39, 1],     # 97=97,5%
    [39, 1],     # 98=97,5%
    [79, 1],     # 99=98,75%
    [2, 0]       # 100=100%
]