MP_id = 742

PQFile = sprintf('DPCPlotPQ_MP%g.txt',MP_id)
ParametersFile = sprintf('DPCPlotParams_MP%g.txt',MP_id)

set bmargin 6
set tmargin 2
set lmargin 20
set rmargin 6

DPC(x,Pb,R,beta,d,Pa) = (x < Pa) ? x*tan(beta)+d : sqrt((d+Pa*tan(beta))**2-((x-Pa)/R)**2)

set angles radians

stats ParametersFile using 1 prefix "Nb" nooutput

array Pb[Nb_records]
array R[Nb_records]
array beta[Nb_records]
array d[Nb_records]
array Pa[Nb_records]

stats ParametersFile using (Pb[$0+1]=$1) prefix "Pbvals" nooutput 
stats ParametersFile using (R[$0+1]=$2) prefix "Rvals" nooutput
stats ParametersFile using (beta[$0+1]=$3) nooutput
stats ParametersFile using (d[$0+1]=$4) prefix "dvals" nooutput
do for [i=1:Nb_records] {
Pa[i] = (Pb[i] - R[i]*d[i])/(1+R[i]*tan(beta[i]))
}

set xrange[0:Pbvals_max*1.03]
set yrange[0:(dvals_max+Pbvals_max*tan(beta[1]))/(1+Rvals_max*tan(beta[1]))*1.03]

plot for [i=1:Nb_records] [0:Pb[i]-0.01] DPC(x,Pb[i],R[i],beta[i],d[i],Pa[i]) w l linewidth 1.5 title sprintf("Pb = %g, R = %g, beta = %g, d = %g",Pb[i],R[i],beta[i],d[i]),\
PQFile u 2:3 title PQFile with points pt 7 pointsize 1.5 lc rgb 'red',\
PQFile u 2:3 notitle with line linewidth 3 lc rgb 'red',\
PQFile u ($2+30):($3+30):0 notitle with labels font 'Helvetica,8'


#set grid back lc rgb 'black' linewidth 1.5
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20" offset -5,0
set xtics font "Helvetica,20"
set ytics font "Helvetica,20"


set title font "Helvetica,30"

set xlabel 'P'
set ylabel 'Q'
