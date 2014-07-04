#!/usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.backends.backend_pdf import PdfPages

from math import cos, sin

def save_pdf(figure, filename='trajecotry'):
    pp = PdfPages('%s.pdf' %filename)
    pp.savefig(figure)
    pp.close()

def movie_clips(width, height, N, region_ts, traj=None):
    K = len(traj)
    for k in xrange(1,K):
        figure = visualize_office(width, height, N, region_ts, traj[1:k])
        figure.savefig('movies/frame%s.png' %k)


def visualize_office(width, height, N, region_ts, traj=None):
    figure = plt.figure()
    axes = figure.add_subplot(1,1,1)    
    #### plot walls
    rect = matplotlib.patches.Rectangle(
            [0,0],
            width, height,
            #facecolor="white",
            fill = False,
            edgecolor="black",
            linewidth=2)
    axes.add_patch(rect)
    for x in xrange(1,N+1):
        for y in [1,N]:
            rect = matplotlib.patches.Rectangle(
                [width*x*1.0/N-width*1.0/N,height*y*1.0/N-height*1.0/N],
                width*0.5/N*2, height*0.5/N*2,
                #facecolor="white",
                fill = False,
                edgecolor="black",
                linewidth=1.5,
                ls="dashed")
            axes.add_patch(rect)
    ### plot balls and baskets, region_ts
    for loc in region_ts.nodes():
        if 'rball' in region_ts.node[loc]['label']:
            rball = matplotlib.patches.Circle(
                (loc[0],loc[1]),
                radius = height*0.13/N,
                facecolor="red",
                #fill = False,
                edgecolor="black",
                linewidth=1,
                ls='solid',
                alpha=0.7)
            axes.add_patch(rball)
        if 'gball' in region_ts.node[loc]['label']:
            gball = matplotlib.patches.Circle(
                (loc[0],loc[1]),
                radius = height*0.13/N,
                facecolor="green",
                #fill = False,
                edgecolor="black",
                linewidth=1,
                ls='solid',
                alpha=0.7)
            axes.add_patch(gball)
        if 'basket' in region_ts.node[loc]['label']:
            basket = matplotlib.patches.Rectangle(
                [loc[0]-width*0.22/N,loc[1]-height*0.22/N],
                width*0.2/N*2,
                height*0.2/N*2,
                facecolor="blue",
                #fill = False,
                edgecolor="black",
                linewidth=1,
                ls='solid',
                alpha=0.7)
            axes.add_patch(basket)
    ### plot traj
    if traj:
        xd = [pose[0] for pose in traj]
        yd = [pose[1] for pose in traj]
        thetad = [pose[2] for pose in traj]
        xdd = [cos(theta) for theta in thetad]
        ydd = [sin(theta) for theta in thetad]
        sample1 = 2
        sample2 = 4
        line = matplotlib.lines.Line2D(
            xd[::sample1], yd[::sample1], 
            marker='o', 
            markersize=3,
            markerfacecolor="black",
            linestyle='-',
            linewidth=1.5,
            color="grey",
            alpha=1)
        axes.quiver(
            xd[::sample2], yd[::sample2],
            xdd[::sample2], 
            ydd[::sample2],
            color='r', 
            units='xy', 
            angles='xy',  
            alpha =1,
            width=13,
            scale = 4.5,
            scale_units='inches',
            )
        axes.add_line(line)
    axes.set_xlim(-2, width)
    axes.set_ylim(-3, height)
    return figure