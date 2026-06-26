import numpy as np
import math
import sys, os
import matplotlib.pyplot as plt
########
cml=sys.argv
plot_file  =      cml[1]  if len(cml)>1  else 'fec0d'
moutput    =      cml[2]  if len(cml)>2  else  'show'
##########
current_dir= os.getcwd()
sys.path.append(current_dir)
exec('import '+plot_file+' as pl')
result=pl.plot_list
legend=pl.key_list
nb_cols=len(result)
print(plot_file)
##########
data_list=[]
for i in range(nb_cols):
	v=[]
	for j in range(len(pl.key_list[i])):
		v.append(np.loadtxt(plot_file+'/pressure/'+pl.plot_list[i][j]+'.txt', comments='#',delimiter=' '))
	data_list.append(v)
##########
plt.rcParams['figure.figsize'] = 12,8
plt.rcParams["axes.formatter.limits"]=-2,3
plt.rcParams["axes.formatter.use_mathtext"]=True
fig_p,axe_p=plt.subplots(nrows=2,ncols=nb_cols,sharex='row', sharey='row',constrained_layout=True)
fig_d,axe_d=plt.subplots(nrows=2,ncols=nb_cols,sharex='row', sharey='row',constrained_layout=True)
fig_v,axe_v=plt.subplots(nrows=2,ncols=nb_cols,sharex='row', sharey='row',constrained_layout=True)
fig_p.suptitle('pressure '+pl.suptitre, fontsize=16)
fig_d.suptitle('shear stress '+pl.suptitre, fontsize=16)
fig_v.suptitle('Velocity norm '+pl.suptitre, fontsize=16)
##########
for i in range(nb_cols):
	axe_p[0,i].set_xlabel('x(m)') 
	axe_p[0,i].set_ylabel('y(m)') 
	axe_p[1,i].set_xlabel('y(m)') 
	axe_p[1,i].set_ylabel('p(Pa)') 
	axe_p[0,i].ticklabel_format(axis='x', style='sci')
	axe_p[1,i].ticklabel_format(axis='x', style='sci')
	axe_p[0,i].ticklabel_format(axis='y', style='sci')
	axe_p[1,i].ticklabel_format(axis='y', style='sci')
	axe_p[0,i].set_title('t='+str(legend[i][0]))
	axe_d[0,i].set_xlabel('x(m)') 
	axe_d[0,i].set_ylabel('y(m)') 
	axe_d[1,i].set_xlabel('y(m)') 
	axe_d[1,i].set_ylabel('sigxy(Pa)') 
	axe_d[0,i].ticklabel_format(axis='x', style='sci')
	axe_d[1,i].ticklabel_format(axis='x', style='sci')
	axe_d[0,i].ticklabel_format(axis='y', style='sci')
	axe_d[1,i].ticklabel_format(axis='y', style='sci')
	axe_d[0,i].set_title('t='+str(legend[i][0]))
	axe_v[0,i].set_xlabel('x(m)') 
	axe_v[0,i].set_ylabel('y(m)') 
	axe_v[1,i].set_xlabel('y(m)') 
	axe_v[1,i].set_ylabel('|v|(m/s)') 
	axe_v[0,i].ticklabel_format(axis='x', style='sci')
	axe_v[1,i].ticklabel_format(axis='x', style='sci')
	axe_v[0,i].ticklabel_format(axis='y', style='sci')
	axe_v[1,i].ticklabel_format(axis='y', style='sci')
	axe_v[0,i].set_title('t='+str(legend[i][0]))
######
	axe_p[0,i].scatter(data_list[i][0][:,0], data_list[i][0][:,1],c=0.5*(data_list[i][0][:,2]+data_list[i][0][:,3]),label ='Pres.')
	axe_d[0,i].scatter(data_list[i][0][:,0], data_list[i][0][:,1],c=     data_list[i][0][:,4],label='Shear St.')
	axe_v[0,i].scatter(data_list[i][0][:,0], data_list[i][0][:,1],c=np.sqrt(np.square(data_list[i][0][:,6])+np.square(data_list[i][0][:,7])),label ='Vel. norm')
	for j in range(1,len(pl.key_list[i])):
		axe_p[1,i].plot(data_list[i][j][:,1],0.5*(data_list[i][j][:,2]+data_list[i][j][:,3]),label='x='+str(legend[i][j]))
		axe_d[1,i].plot(data_list[i][j][:,1],     data_list[i][j][:,4]                      ,label='x='+str(legend[i][j]))
		axe_v[1,i].plot(data_list[i][j][:,1],np.sqrt(np.square(data_list[i][j][:,6])+np.square(data_list[i][j][:,7])),label='x='+str(legend[i][j]))
	axe_p[0,i].legend()
	axe_d[0,i].legend()
	axe_v[0,i].legend()
	axe_p[1,i].legend()
	axe_d[1,i].legend()
	axe_v[1,i].legend()
######
if moutput=='show':
	fig_p.show()
	fig_d.show()
	fig_v.show()
	input() 
else:
	fig_p.savefig(moutput+'_'+'press.pdf'   ,orientation='paysage', papertype='a4', format='pdf')
	fig_d.savefig(moutput+'_'+'shear_st.pdf',orientation='paysage', papertype='a4', format='pdf')
	fig_v.savefig(moutput+'_'+'vel_norm.pdf',orientation='paysage', papertype='a4', format='pdf')
######
plt.close(fig_p)
plt.close(fig_d)
plt.close(fig_v)
