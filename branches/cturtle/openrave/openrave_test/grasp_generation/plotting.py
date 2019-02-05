#!/usr/bin/env python
from numpy import *
import matplotlib.pyplot as plt
from itertools import count, izip
def pause():
    raw_input('press enter to continue...')

if __name__ == '__main__':
    
#    filename = 'log-1285801418.44'
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e=[eval(x) for x in a]
#    plt.plot(k,t)
#    plt.show()
    
#    filename = 'log-1285801804.31'
#    filename = 'log-1285804028.71' # good!
    #filename = 'log-1285804997.36'
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e=[eval(x) for x in a[:-1]]
#    e_min,minindex = min(izip(e, count()))
#    k_min = k[minindex]
#    fig = plt.figure(1)
#    kmax = 80000
#    alpha = .99995
#    fname = '%d'%kmax+'-%.5f'%alpha+'-%.3f'%e_min+'-'+filename+'.png' 
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlim([0,kmax])
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.title('target=data/mug2.kinbody.xml')
#    plt.text(.54*kmax,.4,'alpha=.99995 angle_k=1'+
#            '\ngripper_angle_std=.1'+
#            '\nrotation_z_std=1.57'+
#            '\npos_x_std=.1'+
#            '\npos_y_std=.1'+
#            '\npos_z_std=.1')
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.xlim([0,kmax])
#    plt.ylim([0,2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#    fig.savefig(fname)
#    plt.show()


#    filename = 'log-1285804488.25'
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a]
#    plt.plot(k,t)
#    plt.show()

#    filename = 'log-1285804944.71' #good!
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a]
#    e_min,minindex = min(izip(e, count()))
#    k_min = k[minindex]
#    fig = plt.figure(1)
#    fname = '%d'%kmax+'-%.5f'%alpha+'-%.3f'%e_min+'-'+filename+'.png' 
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlim([0,kmax])
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.title('target=data/mug2.kinbody.xml')
#    plt.text(.54*kmax,.4,'alpha=%.5f'%alpha+' angle_k=1'+
#            '\ngripper_angle_std=%.2f'%gripper_angle_std+
#            '\nrotation_z_std=%.2f'%rotation_z_std+
#            '\npos_x_std=%.2f'%pos_x_std+
#            '\npos_y_std=%.2f'%pos_y_std+
#            '\npos_z_std=%.2f'%pos_z_std)
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.xlim([0,kmax])
#    plt.ylim([0,2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#    fig.savefig(fname)
#    plt.show()

    
#    filename = 'log-1285805380.57'
#    filename = 'log-1285805475.08'
#    filename = 'log-1285805626.56'
#    filename = 'log-1285807564.4' # good!
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    gripper_angle,k,t,e,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a[4:]]
#    e_min,minindex = min(izip(e, count()))
#    k_min = k[minindex]
#    fig = plt.figure(1)
#    fname = '%d'%kmax+'-%.5f'%alpha+'-%.3f'%e_min+'-'+filename+'.png' 
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlim([0,kmax])
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.title('target=data/mug2.kinbody.xml')
#    plt.text(.54*kmax,.4,'alpha=%.5f'%alpha+' angle_k=1'+
#            '\ngripper_angle_std=%.2f'%gripper_angle_std+
#            '\nrotation_z_std=%.2f'%rotation_z_std+
#            '\npos_x_std=%.2f'%pos_x_std+
#            '\npos_y_std=%.2f'%pos_y_std+
#            '\npos_z_std=%.2f'%pos_z_std)
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.xlim([0,kmax])
#    plt.ylim([0,2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#    fig.savefig(fname)
#    plt.show()


#    filename = 'log-1285805467.53' # good!
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a[4:]]
#    e_min,minindex = min(izip(e, count()))
#    k_min = k[minindex]
#    fig = plt.figure(1)
#    fname = '%d'%kmax+'-%.5f'%alpha+'-%.3f'%e_min+'-'+filename+'.png' 
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlim([0,kmax])
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.title('target=data/mug2.kinbody.xml')
#    plt.text(.54*kmax,.4,'alpha=%.5f'%alpha+' angle_k=1'+
#            '\ngripper_angle_std=%.2f'%gripper_angle_std+
#            '\nrotation_z_std=%.2f'%rotation_z_std+
#            '\npos_x_std=%.2f'%pos_x_std+
#            '\npos_y_std=%.2f'%pos_y_std+
#            '\npos_z_std=%.2f'%pos_z_std)
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.xlim([0,kmax])
#    plt.ylim([0,2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#    fig.savefig(fname)
#    plt.show()

    #filename = 'log-1285806780.2'
    #filename = 'log-1285806992.12'
    #filename = 'log-1285807093.92'
    #filename = 'log-1285807155.68'
    #filename = 'log-1285807267.43'
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a[6:]]
#    plt.figure(1)
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.axis([0, kmax, 0, 2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.show()

    #filename = 'log-1285807419.06'
    #filename = 'log-1285807536.09'
    #filename = 'log-1285807936.31' #ok
    #filename = 'log-1285808213.05' #ok
    #filename = 'log-1285808569.81' #good!
    #filename = 'log-1285808881.23' #good!
#    file = open(filename)
#    a=file.readlines()
#    file.close()
#    k,t,e,angle_k,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a[6:]]
#    e_min,minindex = min(izip(e, count()))
#    k_min = k[minindex]
#    fig = plt.figure(1)
#    fname = '%d'%kmax+'-%.5f'%alpha+'-%.3f'%e_min+'-'+filename+'.png' 
#    plt.subplot(211)
#    plt.plot(k,t)
#    plt.xlim([0,kmax])
#    plt.xlabel('k')
#    plt.ylabel('t')
#    plt.title('target=data/mug2.kinbody.xml')
#    plt.text(.54*kmax,.4,'alpha=%.5f'%alpha+' angle_k=%.1f'%angle_k+
#            '\ngripper_angle_std=%.2f'%gripper_angle_std+
#            '\nrotation_z_std=%.2f'%rotation_z_std+
#            '\npos_x_std=%.2f'%pos_x_std+
#            '\npos_y_std=%.2f'%pos_y_std+
#            '\npos_z_std=%.2f'%pos_z_std)
#    plt.subplot(212)
#    plt.plot(k,e)
#    plt.xlim([0,kmax])
#    plt.ylim([0,2.5])
#    plt.xlabel('k')
#    plt.ylabel('e')
#    plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
#            arrowprops=dict(facecolor='black', shrink=0.05),
#            )
#    fig.savefig(fname)
#    plt.show()

    #filename = 'log-1285810574.77'
    #filename = 'log-1285810837.87' #OK
    #filename = 'log-1285811337.6' #ok
    #filename = 'log-1285811847.11' # good
    #filename = 'log-1285812353.21' # good
    #filename = 'log-1285812768.51' # good
    #filename = 'log-1285813687.6'
    #filename = 'log-1285814645.66' # good
    #filename = 'log-1285815361.13'
    #filename = 'log-1285815831.9'
    #filename = 'log-1285816831.95'
    #filename = 'log-1285817594.45'
    #filename = 'log-1285818170.37'
    #filename = 'log-1285826947.47'
    #filename = 'log-1285827028.02'
    #filename = 'log-1285828075.98'
    #filename = 'log-1285829065.56'
    #filename = 'log-1285834332.04'
    #filename = 'log-1285844117.96'
    #filename = 'log-1285846090.77'
    #filename = 'log-1285848267.96'
    #filename = 'log-1285849108.19'
    #filename = 'log-1285851135.3'
    #filename = 'log-1285852390.12'
    #filename = 'log-1285854081.88'
    #filename = 'log-1285855043.75'
    #filename = 'log-1285858205.03'
    #filename = 'log-1285858465.26'
    #filename = 'log-1285859112.88'
    #filename = 'log-1285859168.96'
    #filename = 'log-1285859126.06'
    #filename = 'log-1285859740.64'
    #filename = 'log-1285859914.75'
    #filename = 'log-1285859918.88'
    #filename = 'log-1285859925.94'
    #filename = 'log-1285860676.8'
    #filename = 'log-1285861107.35'
    #filename = 'log-1285861586.05'
    #filename = 'log-1285861667.73'
    #filename = 'log-1285862166.78'
    #filename = 'log-1285862167.33'
    #filename = 'log-1285862171.29'
    #filename = 'log-1285900593.25'
    #filename = 'log-1285900596.07'
    #filename = 'log-1285900660.57'
    #filename = 'log-1285900677.11'
    #filename = 'log-1285900709.72'
    #filename = 'log-1285900719.1'
    #filename = 'log-1285900773.62'
    #filename = 'log-1285900780.06'
    #filename = 'log-1285900843.71'
    #filename = 'log-1285900857.16'
    #filename = 'log-1285905363.85'
    #filename = 'log-1285905419.89'
    #filename = 'log-1285905450.09'
    #filename = 'log-1285905541.94'
    #filename = 'log-1285905585.18'
    #filename = 'log-1285905594.9'
    #filename = 'log-1285905659.07'
    #filename = 'log-1285905686.6'
    #filename = 'log-1285905719.24'
    #filename = 'log-1285905751.04'
    #filename = 'log-1285905828.26'
    #filename = 'log-1285905836.12'
    #filename = 'log-1285906814.58'
    #filename = 'log-1285906826.6'
    #filename = 'log-1285906833.07'
    #filename = 'log-1285906854.85'
    #filename = 'log-1285905898.6'
    #filename = 'log-1285905939.05'
    #filename = 'log-1285906001.96'
    #filename = 'log-1285906332.82'
    #filename = 'log-1285908777.47'
    #filename = 'log-1285908790.29'
    #filename = 'log-1285908815.68'
    #filename = 'log-1285908307.55'
    #filename = 'log-1285908309.51'
    #filename = 'log-1285908310.13'
    #filename = 'log-1285908314.81'
    #filename = 'log-1285908432.65'
    #filename = 'log-1285908432.8'
    #filename = 'log-1285908448.3'
    #filename = 'log-1285908458.04'
    #filename = 'log-1285908782.81'
    #filename = 'log-1285908783.83'
    #filename = 'log-1285908926.5'
    #filename = 'log-1285908936.74'
    #filename = 'log-1285908945.62'
    #filename = 'log-1285910529.42'
    #filename = 'log-1285910540.13'
    #filename = 'log-1285910540.27'
    #filename = 'log-1285911913.79'
    #filename = 'log-1285911932.1'
    #filename = 'log-1285911932.8'
    #filename = 'log-1285911971.06'
    #filename = 'log-1285928529.91'
    #filename = 'log-1285928531.88'
    #filename = 'log-1285928532.82'
    #filename = 'log-1285928537.13'
    #filename = 'log-1285928543.81'
    #filename = 'log-1285940762.01'
    #filename = 'log-1285940798.36'
    #filename = 'log-1285940841.42'
    #filename = 'log-1285940853.67'
    #filename = 'log-1285940858.4'
    #filename = 'log-1285940862.43'
    #filename = 'log-1285940867.16'
    #filename = 'log-1285940876.08'
    #filename = 'log-1285940876.73'
    #filename = 'log-1285940886.7'
    #filename = 'log-1285940903.16'
    #filename = 'log-1285940910.25'
    #filename = 'log-1285940913.85'
    #filename = 'log-1285940925.72'
    #filename = 'log-1285940932.44'
    #filename = 'log-1285940951.97'
    #filename = 'log-1285940965.04'
#    filenames = ['log-1285810574.77','log-1285810837.87','log-1285811337.6','log-1285811847.11',
#                 'log-1285812353.21','log-1285812768.51','log-1285813687.6','log-1285814645.66',
#                 'log-1285815361.13','log-1285815831.9','log-1285816831.95','log-1285817594.45',
#                 'log-1285818170.37','log-1285826947.47','log-1285827028.02','log-1285828075.98',
#                 'log-1285829065.56','log-1285834332.04','log-1285844117.96','log-1285846090.77',
#                 'log-1285848267.96','log-1285849108.19','log-1285851135.3','log-1285852390.12',
#                 'log-1285854081.88','log-1285855043.75','log-1285858205.03','log-1285858465.26',
#                 'log-1285859112.88','log-1285859168.96','log-1285859126.06','log-1285859740.64',
#                 'log-1285859914.75','log-1285859918.88','log-1285859925.94','log-1285860676.8',
#                 'log-1285861107.35','log-1285861586.05','log-1285861667.73','log-1285862166.78',
#                 'log-1285862167.33','log-1285862171.29','log-1285900593.25','log-1285900596.07',
#                 'log-1285900660.57','log-1285900677.11','log-1285900709.72','log-1285900719.1',
#                 'log-1285900773.62','log-1285900780.06','log-1285900843.71','log-1285900857.16',
#                 'log-1285905363.85','log-1285905419.89','log-1285905450.09','log-1285905541.94',
#                 'log-1285905585.18','log-1285905594.9','log-1285905659.07','log-1285905686.6',
#                 'log-1285905719.24','log-1285905751.04','log-1285905828.26','log-1285905836.12',
#                 'log-1285906814.58','log-1285906826.6','log-1285906833.07','log-1285906854.85',
#                 'log-1285905898.6','log-1285905939.05','log-1285906001.96','log-1285906332.82',
#                 'log-1285908777.47','log-1285908790.29','log-1285908815.68','log-1285908307.55',
#                 'log-1285908309.51','log-1285908310.13','log-1285908314.81','log-1285908432.65',
#                 'log-1285908432.8','log-1285908448.3','log-1285908458.04','log-1285908782.81',
#                 'log-1285908783.83','log-1285908926.5','log-1285908936.74','log-1285908945.62',
#                 'log-1285910529.42','log-1285910540.13','log-1285910540.27','log-1285911913.79',
#                 'log-1285911932.1','log-1285911932.8','log-1285911971.06','log-1285928529.91',
#                 'log-1285928531.88','log-1285928532.82','log-1285928537.13','log-1285928543.81',
    filenames = ['log-1285940762.01','log-1285940798.36','log-1285940841.42','log-1285940853.67',
                 'log-1285940858.4','log-1285940862.43','log-1285940867.16','log-1285940876.08',
                 'log-1285940876.73','log-1285940886.7','log-1285940903.16','log-1285940910.25',
                 'log-1285940913.85','log-1285940925.72','log-1285940932.44','log-1285940951.97',
                 'log-1285940965.04']
    for filename in filenames:
        print filename
        file = open(filename)
        a=file.readlines()
        file.close()
        targetfile = a[8]
        k,t,e,angle_k,kmax,alpha,gripper_angle_std,rotation_z_std,pos_x_std,pos_y_std,pos_z_std=[eval(x) for x in a[9:]]
        e_min,minindex = min(izip(e, count()))
        k_min = k[minindex]
        fig = plt.figure()
        fname = '%d'%kmax+\
            '-%.5f'%alpha+\
            '-%.2f'%gripper_angle_std+\
            '-%.2f'%rotation_z_std+\
            '-%.2f'%pos_x_std+\
            '-'+filename+'.png' 
            #'-'+filename+'.pdf'
        plt.subplot(211)
        plt.plot(k,t)
        plt.xlim([0,kmax])
        plt.xlabel('k')
        plt.ylabel('t')
        plt.title('target=%s'%targetfile)
        plt.text(.54*kmax,.4,'alpha=%.5f'%alpha+' angle_k=%.1f'%angle_k+
                '\ngripper_angle_std=%.2f'%gripper_angle_std+
                '\nrotation_z_std=%.2f'%rotation_z_std+
                '\npos_x_std=%.2f'%pos_x_std+
                '\npos_y_std=%.2f'%pos_y_std+
                '\npos_z_std=%.2f'%pos_z_std)
        plt.subplot(212)
        plt.plot(k,e)
        plt.xlim([0,kmax])
        plt.ylim([0,2.5])
        plt.xlabel('k')
        plt.ylabel('e')
        plt.annotate('e_min=%.3f'%e_min+'\nk=%d'%k_min, xy=(k_min, e_min), xytext=(k_min*.9, 1.5),
                arrowprops=dict(facecolor='black', shrink=0.05),
                )
        fig.savefig(fname)
        #plt.show()
